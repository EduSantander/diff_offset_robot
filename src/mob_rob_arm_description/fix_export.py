#!/usr/bin/env python3
"""Uso: python3 fix_export.py <urdf_exportado> <xacro_salida> <nombre_del_paquete>"""
import re
import sys

COLUMN_JOINT = "column_joint"     # junta prismatica (nombre en tu URDF)
TURRET_JOINT = "top_base_joint"   # junta continua de la torreta

src, dst, pkg = sys.argv[1], sys.argv[2], sys.argv[3]
s = open(src, encoding="utf-8").read().replace("\r\n", "\n")

# 1) Cabecera xacro
s, n = re.subn(r'<robot\s+name="([^"]+)">',
               r'<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="\1">', s, count=1)
assert n == 1, "No encontre la etiqueta <robot name=...>"

# 2) Rutas de las mallas -> tu paquete
s = re.sub(r"package://[^/]+/", f"package://{pkg}/", s)

# 3) Quitar <axis xyz="0 0 0"/> (sobra en juntas fijas)
s = re.sub(r'\n\s*<axis\s+xyz="0 0 0"\s*/>', "", s)

# 4) Columna prismatica: limites realistas + dinamica
pat = re.compile(r'(<joint\s+name="%s"\s+type="prismatic">.*?)(</joint>)' % COLUMN_JOINT, re.S)
m = pat.search(s)
assert m, f"No encontre la junta prismatica '{COLUMN_JOINT}'"
body = re.sub(r'effort="[^"]*"', 'effort="${column_effort}"', m.group(1))
body = re.sub(r'velocity="[^"]*"', 'velocity="${column_velocity}"', body)
body += '  <dynamics damping="${column_damping}" friction="${column_friction}" />\n  '
s = s[:m.start()] + body + m.group(2) + s[m.end():]

# 5) Torreta continua: dinamica
pat = re.compile(r'(<joint\s+name="%s"\s+type="continuous">.*?)(</joint>)' % TURRET_JOINT, re.S)
m = pat.search(s)
assert m, f"No encontre la junta continua '{TURRET_JOINT}'"
body = m.group(1) + '  <dynamics damping="${turret_damping}" friction="${turret_friction}" />\n  '
s = s[:m.start()] + body + m.group(2) + s[m.end():]

# 6) Propiedades ajustables
props = '''
  <!-- ===== Parametros ajustables (fix_export.py) ===== -->
  <xacro:property name="column_effort" value="2000"/>   <!-- N: UR10 + carga + columna -->
  <xacro:property name="column_velocity" value="0.1"/>  <!-- m/s -->
  <xacro:property name="column_damping" value="200"/>
  <xacro:property name="column_friction" value="5"/>
  <xacro:property name="turret_damping" value="20"/>
  <xacro:property name="turret_friction" value="1"/>
'''
s = re.sub(r'(<robot [^>]*>\n)', lambda mm: mm.group(1) + props, s, count=1)

open(dst, "w", encoding="utf-8").write(s)
print(f"OK -> {dst}")