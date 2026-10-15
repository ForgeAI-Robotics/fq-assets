"""Release and main asset versions must remain independent."""
import json,tempfile,unittest
from pathlib import Path
from server import Catalog

class VersionTest(unittest.TestCase):
    def test_main_asset_newer_than_release(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);sha='a'*40;snap=root/'snapshots'/sha;snap.mkdir(parents=True)
            (snap/'catalog.json').write_text(json.dumps({'main_commit':sha,'assets':[{'id':'fq:objects/coke_cn','version':'0.1.1'}]}))
            catalog=Catalog(root,root,60)
            def fake_run(args,*rest,**kwargs):
                if args[:2]==['git','rev-parse']:return sha
                if args[1]=='api':return json.dumps({'tag_name':'v0.1.0','html_url':'https://github.com/ForgeAI-Robotics/fq-assets/releases/tag/v0.1.0'})
                return ''
            catalog.run=fake_run;catalog.sync();response=catalog.response()
            self.assertEqual(response['latest_release']['tag'],'v0.1.0')
            self.assertEqual(response['assets'][0]['version'],'0.1.1')
            self.assertEqual(response['main_commit'],sha)
            restored=Catalog(root,root,60).response()
            self.assertEqual(restored['assets'][0]['version'],'0.1.1')
            self.assertEqual(restored['latest_release']['tag'],'v0.1.0')
if __name__=='__main__':unittest.main()
