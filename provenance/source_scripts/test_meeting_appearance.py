"""Portable materials, approximate lighting, and static USD export.

The textures are deterministic procedural asset data, not surveyed materials.
USD and MJCF share the same box coordinates and texture images.
"""
from pathlib import Path
import random
import xml.etree.ElementTree as ET
from PIL import Image

PALETTE = {'wall': (.87,.87,.82), 'furniture': (.78,.75,.66),
           'door': (.7,.64,.53), 'blocked': (.65,.36,.36), 'floor': (.4,.4,.4),
           'stone':(.65,.66,.66),'frame':(.035,.04,.045),'glass':(.83,.91,.92),
           'light':(1,1,.96),'logo':(.9,.9,.88),'ceiling':(.94,.94,.92)}
TEXTURES={'floor':'carpet_photo_v1.png','wall':'wall.png','door':'wood_photo_v1.png','stone':'stone_photo_v1.png'}
LIGHTS = [(2,10.4,2.65),(4.4,10.4,2.65),(12,10.3,2.65),(15,10.3,2.65),
          (11.3,3.5,2.65),(11.3,6.2,2.65),(7.3,11.3,2.65)]


def textures(out):
    folder=out/'textures'
    folder.mkdir(exist_ok=True)
    rng=random.Random(314)
    for kind, base in [('floor',(191,194,193)),('wall',(231,229,220))]:
        im=Image.new('RGB',(512,512))
        pixels=[]
        for y in range(512):
            for x in range(512):
                delta=rng.choice([-2,-1,0,0,0,1,2])
                if kind=='floor' and (x<3 or y<3):
                    pixels.append((151,156,154))
                else:
                    pixels.append(tuple(v+delta for v in base))
        im.putdata(pixels)
        im.save(folder/f'{kind}.png')


def add_mjcf(root,world,out):
    textures(out)
    asset=ET.SubElement(root,'asset')
    for kind,filename in TEXTURES.items():
        ET.SubElement(asset,'texture',name=f'{kind}_texture',type='2d' if kind in ('floor','stone') else 'cube',file=f'textures/{filename}')
    for kind,color in PALETTE.items():
        props=dict(name=f'{kind}_material',rgba=' '.join(map(str,(*color,1))),specular='.12',shininess='.15',reflectance='0')
        if kind in TEXTURES:
            props.update(texture=f'{kind}_texture',rgba='1 1 1 1',texuniform='true',texrepeat='1.666667 1.666667' if kind=='stone' else '1 1')
        if kind=='glass':props.update(rgba='.83 .91 .92 .20',specular='.7',shininess='.8')
        if kind in ('stone','logo'):props.update(specular='.55',shininess='.65')
        if kind=='floor':props.update(specular='0',shininess='0')
        if kind=='light':props.update(emission='1')
        if kind=='ceiling':props.update(emission='.15')
        ET.SubElement(asset,'material',**props)
    for light in list(world.findall('light')):
        world.remove(light)
    for i,pos in enumerate(LIGHTS):
        ET.SubElement(world,'light',name=f'ceiling_light_{i}',pos=' '.join(map(str,pos)),dir='0 0 -1',directional='false',diffuse='.8 .78 .72',specular='.12 .12 .12',ambient='.08 .08 .08',castshadow='true')
        ET.SubElement(world,'geom',name=f'downlight_fixture_{i}',type='cylinder',pos=' '.join(map(str,pos)),size='.09 .012',material='light_material',contype='0',conaffinity='0')
    ET.SubElement(world,'light',name='lobby_panel_light',pos='7.5 11.3 2.55',dir='0 0 -1',diffuse='.9 .9 .88',ambient='.15 .15 .15',castshadow='true')
    head=root.find('./visual/headlight')
    head.set('ambient','.5 .5 .5')
    head.set('diffuse','.5 .5 .5')


def vector(values):
    return '('+', '.join(f'{v:.6f}' for v in values)+')'


def usd_mesh(name, center, half, material, collision=True, visible=True):
    x,y,z=center; a,b,c=half
    vertices=[(x-a,y-b,z-c),(x+a,y-b,z-c),(x+a,y+b,z-c),(x-a,y+b,z-c),
              (x-a,y-b,z+c),(x+a,y-b,z+c),(x+a,y+b,z+c),(x-a,y+b,z+c)]
    faces=[(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]
    period=.6 if material=='stone' else 1.0
    uv=[]
    for k,face in enumerate(faces):
        axes=(0,1) if k<2 else ((0,2) if k in (2,4) else (1,2))
        uv.extend((vertices[i][axes[0]]/period,vertices[i][axes[1]]/period) for i in face)
    result=f'''    def Mesh "{name}" (
        prepend apiSchemas = ["MaterialBindingAPI"]
    ) {{
        token visibility = "{'inherited' if visible else 'invisible'}"
        point3f[] points = [{', '.join(map(vector,vertices))}]
        int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]
        int[] faceVertexIndices = [{', '.join(str(i) for f in faces for i in f)}]
        uniform token subdivisionScheme = "none"
        texCoord2f[] primvars:st = [{', '.join(map(vector,uv))}] (
            interpolation = "faceVarying"
        )
        rel material:binding = </World/Looks/{material}>
        rel material:binding:physics = </World/Looks/contact>
    }}
'''
    if not collision:return result
    return result+f'''
    def Cube "{name}_collision" (
        prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"]
    ) {{
        double size = 2
        double3 xformOp:translate = {vector(center)}
        double3 xformOp:scale = {vector(half)}
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        token visibility = "invisible"
        bool physics:collisionEnabled = true
        rel material:binding:physics = </World/Looks/contact>
    }}
'''


def export_usd(out,data):
    text=['#usda 1.0\n(\n    defaultPrim = "World"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\ndef Xform "World" {\n']
    text.append('    def PhysicsScene "Physics" {\n        vector3f physics:gravityDirection = (0, 0, -1)\n        float physics:gravityMagnitude = 9.81\n    }\n')
    text.append('    def Scope "Looks" {\n')
    for kind,color in PALETTE.items():
        text.append(f'''        def Material "{kind}" {{
            token outputs:surface.connect = </World/Looks/{kind}/Surface.outputs:surface>
            def Shader "Surface" {{
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = {vector(color)}
                float inputs:roughness = {'.12' if kind in ('glass','stone','logo') else '.85'}
                float inputs:metallic = 0
                token outputs:surface
''')
        if kind=='glass':text.append('                float inputs:opacity = .20\n                float inputs:ior = 1.5\n')
        if kind=='light':text.append('                color3f inputs:emissiveColor = (1, 1, .96)\n')
        if kind=='ceiling':text.append('                color3f inputs:emissiveColor = (.15, .15, .14)\n')
        if kind in TEXTURES:
            text.append(f'                color3f inputs:diffuseColor.connect = </World/Looks/{kind}/Texture.outputs:rgb>\n')
        text.append('            }\n')
        if kind in TEXTURES:
            text.append(f'''            def Shader "UV" {{
                uniform token info:id = "UsdPrimvarReader_float2"
                token inputs:varname = "st"
                float2 outputs:result
            }}
            def Shader "Texture" {{
                uniform token info:id = "UsdUVTexture"
                asset inputs:file = @textures/{TEXTURES[kind]}@
                token inputs:sourceColorSpace = "sRGB"
                token inputs:wrapS = "repeat"
                token inputs:wrapT = "repeat"
                float2 inputs:st.connect = </World/Looks/{kind}/UV.outputs:result>
                float3 outputs:rgb
            }}
''')
        text.append('        }\n')
    text.append('''        def Material "contact" (prepend apiSchemas = ["PhysicsMaterialAPI"]) {
            float physics:staticFriction = .7
            float physics:dynamicFriction = .7
            float physics:restitution = 0
        }
    }
''')
    text.append(usd_mesh('floor',[8.55,6.2,-.05],[8.65,6.3,.05],'floor'))
    for box in data['boxes']:
        text.append(usd_mesh(box['name'],box['center'],box['half'],box['kind'],box.get('collision',True),box.get('visible',True)))
    for i,pos in enumerate(LIGHTS):
        text.append(f'''    def DiskLight "ceiling_light_{i}" {{
        float inputs:intensity = 25000
        float inputs:radius = .15
        bool inputs:enableColorTemperature = true
        float inputs:colorTemperature = 4800
        double3 xformOp:translate = {vector(pos)}
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }}
    def Cylinder "downlight_fixture_{i}" (prepend apiSchemas = ["MaterialBindingAPI"]) {{
        double radius = .09
        double height = .024
        uniform token axis = "Z"
        double3 xformOp:translate = {vector(pos)}
        uniform token[] xformOpOrder = ["xformOp:translate"]
        rel material:binding = </World/Looks/light>
    }}
''')
    text.append('''    def RectLight "lobby_panel_light" {
        float inputs:intensity = 5000
        float inputs:width = 3.2
        float inputs:height = 1.4
        color3f inputs:color = (1, 1, .98)
        double3 xformOp:translate = (7.5, 11.3, 2.55)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
    def DomeLight "ambient" {
        float inputs:intensity = 750
    }
}
''')
    (out/'scene.usda').write_text(''.join(text),encoding='utf-8')
