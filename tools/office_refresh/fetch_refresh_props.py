from pathlib import Path
import urllib.request,json,zipfile,io,hashlib
S=Path('/project/office_assets/refresh');S.mkdir(parents=True,exist_ok=True)
items=[('gso_white_mug','Threshold_Porcelain_Coffee_Mug_All_Over_Bead_White'),('gso_blue_mug','Cole_Hardware_Mug_Classic_Blue')]
for slug,name in items:
 d=S/slug;d.mkdir(exist_ok=True);url='https://fuel.gazebosim.org/1.0/GoogleResearch/models/'+name
 meta=json.load(urllib.request.urlopen(url));assert meta['license_id']==2
 data=urllib.request.urlopen(url+'/tip/'+name+'.zip',timeout=180).read()
 with zipfile.ZipFile(io.BytesIO(data)) as z:
  for p in z.namelist():assert (d/p).resolve().is_relative_to(d.resolve())
  z.extractall(d)
 (d/'source.json').write_text(json.dumps(dict(url=url,metadata=meta,sha256=hashlib.sha256(data).hexdigest(),license='CC-BY-4.0')))
 print('FETCHED',slug,flush=True)
d=S/'khronos_water_bottle';d.mkdir(exist_ok=True)
base='https://raw.githubusercontent.com/KhronosGroup/glTF-Sample-Assets/main/Models/WaterBottle/'
for name,url in [('source.glb',base+'glTF-Binary/WaterBottle.glb'),('LICENSE_source.md',base+'README.md')]:
 data=urllib.request.urlopen(url,timeout=180).read();(d/name).write_bytes(data)
(d/'source.json').write_text(json.dumps(dict(url=base,license='CC0-1.0',author='Microsoft',sha256=hashlib.sha256((d/'source.glb').read_bytes()).hexdigest())))
print('FETCHED water bottle',flush=True)
