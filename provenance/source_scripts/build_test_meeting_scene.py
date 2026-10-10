"""Build a simplified, estimated office scene from the supplied 3.14 PDF.

Door clear width and conference table width are user supplied (0.80/1.20 m).
Other geometry remains estimated from the drawing and reference photographs.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from test_meeting_appearance import add_mjcf, export_usd

ROOT = Path(__file__).resolve().parents[1]


def build(door_width=0.80):
    if not 0.6 <= door_width <= 1.3:
        raise ValueError('door width must be between 0.6 and 1.3 metres')
    out = ROOT / 'simulation' / 'test_meeting'
    out.mkdir(parents=True, exist_ok=True)
    # Coordinates traced on a 1888-pixel-wide rendering, origin at lower left.
    scale = 88.0
    xy = lambda x, y: ((x - 182) / scale, (1225 - y) / scale)
    boxes = []

    def box(name, x0, y0, x1, y1, height=2.7, kind='wall', base=0, collision=True):
        a, b = xy(x0, y0), xy(x1, y1)
        boxes.append(dict(name=name, center=[(a[0]+b[0])/2, (a[1]+b[1])/2, base+height/2],
                          half=[abs(b[0]-a[0])/2, abs(b[1]-a[1])/2, height/2], kind=kind,collision=collision))

    # Outer footprint, with exterior openings intentionally closed for this task.
    box('north_west', 177, 129, 755, 139)
    box('north_east', 878, 129, 1692, 139)
    box('exit_glass_left',755,132,816.5,136,2.1,'glass')
    box('exit_glass_right',816.5,132,878,136,2.1,'glass')
    box('exit_header',755,129,878,139,.6,'wall',2.1)
    for i,x in enumerate([755,816.5,878]):box(f'exit_frame_{i}',x-2,129,x+2,139,2.1,'frame')
    for i,x in enumerate([805,826]):box(f'exit_handle_{i}',x-1,140,x+1,144,.7,'frame',.8,False)
    box('south', 177, 1220, 1692, 1230)
    box('west', 177, 139, 187, 1220)
    box('east', 1682, 139, 1692, 1220)
    door_top = 149
    door_bottom = door_top + door_width*scale
    box('meeting_door_upper_jamb', 667, 139, 677, door_top)
    box('meeting_east_wood',667,door_bottom,677,300,2.7,'door')
    box('meeting_east_wall',667,300,677,486)
    box('meeting_east_stub',667,486,677,535)
    # Put the jamb entirely outside the opening: door_width is CLEAR width.
    box('meeting_door_lower_frame',667,door_bottom,677,door_bottom+4,2.7,'frame')
    box('meeting_door_lintel', 667, door_top, 677, door_bottom, .6, base=2.1)
    box('meeting_south_wall',187,486,667,496)
    box('meeting_cabinet', 218, 496, 667, 539, 1.0, 'furniture')
    box('logo_wall', 677, 315, 1029, 330,2.7,'logo')
    box('storage_machine_partition', 851, 330, 861, 535)
    box('storage_south', 764, 530, 888, 540)
    box('machine_south', 958, 548, 1029, 558)
    box('machine_east', 1019, 418, 1029, 548)
    box('column_machine', 888, 487, 958, 557)
    box('column_inner', 882, 386, 935, 439)
    box('office_north', 1336, 548, 1682, 558)
    box('meeting_table', 204, 258, 557, 372, .75, 'furniture')
    boxes[-1]['half'][1]=.60  # User confirmed long conference table width: 1.20 m.
    # Photo 4 entrance/lobby finish is distinct from the carpeted test area.
    # Thin non-colliding finish above the common floor avoids a simulated step.
    box('entrance_stone_finish',677,139,1029,315,.002,'stone',0,False)
    for i,x in enumerate(range(731,1029,53)):
        box(f'lobby_grout_x_{i}',x,139,x+.14,315,.0005,'wall',.002,False)
    for i,y in enumerate(range(192,315,53)):
        box(f'lobby_grout_y_{i}',677,y,1029,y+.14,.0005,'wall',.002,False)
    for i,x in enumerate(range(1070,1650,100)):
        box(f'wall_panel_joint_{i}',x,139,x+.18,139.5,2.65,'frame',0,False)
    box('meeting_light_strip',192,144,655,147,.025,'light',2.64,False)
    box('lobby_light_strip',682,304,1018,307,.025,'light',2.64,False)
    box('ceiling_upper',187,139,1682,558,.08,'ceiling',2.7,False)
    box('ceiling_corridor',1029,558,1336,1220,.08,'ceiling',2.7,False)
    box('lobby_luminous_panel',690,155,1010,298,.025,'light',2.61,False)
    # Red-marked strip in the user's annotated plan: east of machine room,
    # west of the right-hand offices. The former desks here have been removed.
    box('temporary_unreachable_southwest', 182, 558, 1029, 1225, 2.7, 'blocked')
    boxes[-1]['visible']=False
    box('temporary_unreachable_southeast', 1336, 558, 1687, 1225, 2.7, 'blocked')
    boxes[-1]['visible']=False
    # Glass perimeter is visual; the unchanged solid volumes enforce exclusion.
    for zone,x0,x1 in [('sw',182,1029),('se',1336,1687)]:
        y0,y1=558,1225
        for side,a,b,c,d in [('north',x0,y0,x1,y0),('south',x0,y1,x1,y1),
                              ('west',x0,y0,x0,y1),('east',x1,y0,x1,y1)]:
            horizontal=b==d
            length=abs(c-a) if horizontal else abs(d-b)
            n=max(1,round(length/(1.2*scale)))
            for i in range(n):
                t0,t1=i/n,(i+1)/n
                u,v=a+(c-a)*t0,b+(d-b)*t0
                w,z=a+(c-a)*t1,b+(d-b)*t1
                box(f'blocked_{zone}_{side}_glass_{i}',u-(0 if horizontal else 1),v-(1 if horizontal else 0),w+(0 if horizontal else 1),z+(1 if horizontal else 0),2.7,'glass',0,False)
            for i in range(n+1):
                u,v=a+(c-a)*i/n,b+(d-b)*i/n
                box(f'blocked_{zone}_{side}_post_{i}',u-1.5,v-1.5,u+1.5,v+1.5,2.7,'frame',0,False)
            for tag,height,base in [('base',.045,0),('top',.045,2.655)]:
                box(f'blocked_{zone}_{side}_{tag}',a-(0 if horizontal else 1.5),b-(1.5 if horizontal else 0),c+(0 if horizontal else 1.5),d+(1.5 if horizontal else 0),height,'frame',base,False)
    # Door at 90 degrees, opening into the meeting room, leaf against upper jamb.
    box('meeting_open_leaf', 667-door_width*scale, 145, 667, 149, 2.1, 'door')
    door_center = list(xy(672, (door_top+door_bottom)/2))
    spawn = [13.0, 10.2]
    goal = [4.65, door_center[1]]
    route = [spawn, [7.0, door_center[1]], [6.2, door_center[1]], door_center, goal]
    root = ET.Element('mujoco', model='PDF estimated robot-test to meeting-room')
    ET.SubElement(root, 'option', gravity='0 0 -9.81')
    ET.SubElement(root, 'statistic', center='8.5 6.2 0', extent='19')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual,'global', offwidth='1400', offheight='1000')
    ET.SubElement(visual,'headlight', ambient='.45 .45 .45', diffuse='.7 .7 .7')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world,'light',pos='8 6 15',dir='0 0 -1',directional='true')
    add_mjcf(root,world,out)
    ET.SubElement(world,'geom',name='floor',type='box',pos='8.55 6.2 -.05',size='8.65 6.3 .05',material='floor_material',friction='.7 .005 .0001')
    for b in boxes:
        geom=ET.SubElement(world, 'geom', name=b['name'], type='box',
                      pos=' '.join(f'{v:.5f}' for v in b['center']),
                      size=' '.join(f'{v:.5f}' for v in b['half']),material=f"{b['kind']}_material",friction='.7 .005 .0001',contype='1' if b['collision'] else '0',conaffinity='1' if b['collision'] else '0')
        if not b.get('visible',True):geom.set('rgba','0 0 0 0')
    for name, pos, color in [('start', spawn, '.1 .7 .3 .8'),('goal',goal,'.9 .3 .1 .8')]:
        ET.SubElement(world,'site',name=name,pos=f'{pos[0]} {pos[1]} .025',size='.18',rgba=color)
    ET.indent(root)
    ET.ElementTree(root).write(out/'scene.xml',encoding='utf-8',xml_declaration=True)
    # The lintel is above the planar navigation scan, so exclude it from 2D obstacles.
    rectangles = [[*b['center'][:2],*b['half'][:2]] for b in boxes if b['collision'] and b['center'][2]-b['half'][2]<.5]
    data = dict(source='平面布置图--调整3.14.pdf, page 1', units='metres',
                accuracy='layout traced; user supplied door clear width 0.80 m and conference table width 1.20 m',
                photo_revision='2026-10-09: carpet, entrance stone, pale wood, black frames, glass and downlights',
                appearance_assumptions='user revision: lower exclusion zones have black-framed glass boundaries; meeting partitions are solid; other finish boundaries approximate',
                calibration='approximately 88 px/m from repeated 1200 mm desk dimension',
                assumptions=dict(door_width=door_width, door_width_definition='clear opening',meeting_table_width=1.2,door_height=2.1, wall_height=2.7,
                                 door_state='fixed open 90 degrees', chairs='omitted',
                                 exterior_doors='closed', furniture='solid simplified boxes',
                                 lower_office='red-marked eastern desk row removed; remaining lower areas blocked'),
                corridor=dict(bounds=[(1029-182)/scale,5/scale,(1336-182)/scale,(1225-558)/scale],
                              width=(1336-1029)/scale,
                              test_route=[[11.3,1.0],[11.3,8.6],spawn],
                              source='user red rectangle, 2026-10-09; approximate edges aligned to adjacent walls'),
                exclusion_zone=dict(names=['temporary_unreachable_southwest','temporary_unreachable_southeast'],
                                    status='corrected from user annotated plan',
                                    interpretation='lower areas outside the red-marked north-south corridor',
                                    bounds_list=[[0,0,(1029-182)/scale,(1225-558)/scale],[(1336-182)/scale,0,(1687-182)/scale,(1225-558)/scale]],
                                    temporary=True, physical_structure=False),
                resolution=.05, origin=[-1,-1], spawn=spawn, goal=goal,
                door_center=door_center, suggested_waypoints=route, rectangles=rectangles, boxes=boxes)
    (out/'geometry.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
    export_usd(out,data)
    # Optional G1 wrapper: requires the existing Linux project's robot dependency.
    wrapper = ET.Element('mujoco',model='Estimated office with G1')
    ET.SubElement(wrapper,'include',file='../../.deps/unitree_rl_gym/resources/robots/g1_description/g1_12dof.xml')
    ET.SubElement(wrapper,'include',file='scene.xml')
    ET.indent(wrapper)
    ET.ElementTree(wrapper).write(out/'scene_g1.xml',encoding='utf-8',xml_declaration=True)
    print(out)
    return out


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--door-width',type=float,default=.80)
    build(parser.parse_args().door_width)
