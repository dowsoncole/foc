import xml.etree.ElementTree as ET
from xml.dom import minidom

f = r'C:\Users\penghao\Desktop\Servo_9948\G431_base_code\software\servo_v1\MDK-ARM\servo_v1.uvprojx'

tree = ET.parse(f)
root = tree.getroot()

raw = ET.tostring(root, encoding='unicode', xml_declaration=False)
dom = minidom.parseString(raw)
pretty = dom.toprettyxml(indent='  ')

lines = pretty.split('\n')
out_lines = []
for line in lines:
    if line.strip().startswith('<?xml'):
        continue
    out_lines.append(line)

header = '<?xml version="1.0" encoding="UTF-8" standalone="no" ?>'
body = '\r\n'.join(out_lines)
content = header + '\r\n' + body + '\r\n'

with open(f, 'wb') as fp:
    fp.write(b'\xef\xbb\xbf')
    fp.write(content.encode('utf-8'))

print(f'Done, {len(content.encode("utf-8"))+3} bytes written')
