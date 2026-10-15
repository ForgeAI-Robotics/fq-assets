"""Small LAN asset viewer. Atomic main snapshots, no editing endpoints."""
from pathlib import Path
from http.server import ThreadingHTTPServer,BaseHTTPRequestHandler
from urllib.parse import urlsplit,unquote
import argparse,datetime,json,mimetypes,os,subprocess,threading,time,traceback

BASE=Path(__file__).resolve().parent
def now():return datetime.datetime.now(datetime.timezone.utc).isoformat()

class Catalog:
    def __init__(self,repo,state,interval):
        self.repo=repo.resolve();self.state=state.resolve();self.interval=interval
        self.state.mkdir(parents=True,exist_ok=True);self.lock=threading.Lock()
        self.catalog=None;self.status={'syncing':True,'last_checked':None,'last_success':None,'error':None}
        self.release={'tag':'unknown','url':None};self.release_time=0
        if (self.state/'current.json').exists():
            current=json.loads((self.state/'current.json').read_text())
            self.catalog=json.loads((self.state/'snapshots'/current['commit']/'catalog.json').read_text())
            self.status.update(current.get('status',{}));self.release=current.get('release',self.release)
    def run(self,args,cwd=None,timeout=120,env=None):
        result=subprocess.run(args,cwd=cwd,timeout=timeout,env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        if result.returncode:raise RuntimeError(result.stderr[-1500:] or result.stdout[-1500:])
        return result.stdout.strip()
    def sync(self):
        with self.lock:self.status.update(syncing=True,last_checked=now())
        try:
            self.run(['git','fetch','origin','main','--tags'],self.repo,60)
            sha=self.run(['git','rev-parse','origin/main'],self.repo)
            if time.time()-self.release_time>300:
                try:
                    raw=self.run([str(Path.home()/'.local/bin/gh'),'api','repos/ForgeAI-Robotics/fq-assets/releases/latest'],timeout=35)
                    r=json.loads(raw)
                    with self.lock:self.release={'tag':r['tag_name'],'url':r['html_url']}
                    self.release_time=time.time()
                except Exception as e:
                    print('RELEASE_REFRESH_ERROR',str(e),flush=True)
            snap=self.state/'snapshots'/sha
            if not (snap/'catalog.json').is_file():
                wt=self.state/'worktrees'/sha;wt.parent.mkdir(parents=True,exist_ok=True)
                if not wt.exists():
                    self.run(['git','worktree','add','--detach',str(wt),sha],self.repo,90,dict(os.environ,GIT_LFS_SKIP_SMUDGE='1'))
                self.run(['git','lfs','pull','origin','main'],wt,240)
                self.run(['git','lfs','checkout'],wt,120)
                snap.mkdir(parents=True,exist_ok=True)
                self.run(['docker','run','--rm','--network','none','--user',f'{os.getuid()}:{os.getgid()}',
                    '-e','PYTHONPATH=/deps','-v',f'{Path.home()}/.local/share/fq-assets-validation-deps:/deps:ro',
                    '-v',f'{wt}:/assets:ro','-v',f'{BASE}:/app:ro','-v',f'{snap}:/out',
                    'g1-navigation-sim:20261008','python3','/app/export_assets.py','--root','/assets','--out','/out','--commit',sha],timeout=240)
            data=json.loads((snap/'catalog.json').read_text())
            with self.lock:
                self.catalog=data;self.status.update(syncing=False,last_success=now(),error=None)
                record={'commit':sha,'status':self.status,'release':self.release}
                temp=self.state/'current.tmp';temp.write_text(json.dumps(record));temp.replace(self.state/'current.json')
            print('SYNC_OK',sha,flush=True)
        except Exception as e:
            print('SYNC_ERROR',str(e),flush=True)
            with self.lock:self.status.update(syncing=False,error='暂时无法同步 main，继续展示上次成功同步的内容。')
    def loop(self):
        while True:
            self.sync();time.sleep(self.interval)
    def response(self):
        with self.lock:
            return {**(self.catalog or {'assets':[],'main_commit':None}),'latest_release':self.release,
                    'sync':dict(self.status),'sync_interval_seconds':self.interval,'repository':'https://github.com/ForgeAI-Robotics/fq-assets'}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--repo',type=Path,required=True);ap.add_argument('--state',type=Path,required=True)
    ap.add_argument('--host',default='127.0.0.1');ap.add_argument('--port',type=int,default=18780);ap.add_argument('--interval',type=int,default=60)
    args=ap.parse_args();catalog=Catalog(args.repo,args.state,args.interval)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,fmt,*a):
            if a and str(a[0]).startswith('GET /api'):return
            super().log_message(fmt,*a)
        def do_GET(self):
            path=unquote(urlsplit(self.path).path)
            if path in ('/api/catalog','/healthz'):
                data=catalog.response() if path=='/api/catalog' else {'ok':True,'ready':catalog.catalog is not None}
                body=json.dumps(data,ensure_ascii=False).encode();self.send_response(200)
                self.send_header('Content-Type','application/json; charset=utf-8');self.send_header('Cache-Control','no-store')
                self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body);return
            if path.startswith('/data/'):
                root=catalog.state/'snapshots';rel=path[6:]
            else:root=BASE/'web';rel=path.lstrip('/') or 'index.html'
            target=(root/rel).resolve()
            if not target.is_relative_to(root.resolve()) or not target.is_file() or any(p.startswith('.') for p in Path(rel).parts):
                self.send_error(404);return
            if target.suffix not in ('.html','.css','.js','.json','.bin','.png','.svg','.txt'):
                self.send_error(404);return
            zipped=Path(str(target)+'.gz')
            compressed='gzip' in self.headers.get('Accept-Encoding','') and zipped.is_file()
            actual=zipped if compressed else target
            self.send_response(200)
            mime={'.js':'text/javascript','.bin':'application/octet-stream'}.get(target.suffix,mimetypes.guess_type(target.name)[0] or 'application/octet-stream')
            self.send_header('Content-Type',mime);self.send_header('Content-Length',str(actual.stat().st_size))
            self.send_header('X-Content-Type-Options','nosniff')
            self.send_header('Cache-Control','public, max-age=31536000, immutable' if path.startswith('/data/') else 'no-cache')
            self.send_header('Vary','Accept-Encoding')
            if compressed:self.send_header('Content-Encoding','gzip')
            self.end_headers()
            try:
                with actual.open('rb') as f:
                    while chunk:=f.read(256*1024):self.wfile.write(chunk)
            except (BrokenPipeError,ConnectionResetError):pass
    threading.Thread(target=catalog.loop,daemon=True).start()
    server=ThreadingHTTPServer((args.host,args.port),Handler)
    print(f'FQ Assets http://{args.host}:{args.port}',flush=True);server.serve_forever()
if __name__=='__main__':main()
