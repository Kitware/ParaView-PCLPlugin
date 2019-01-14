#!/usr/bin/env python3

import argparse
import re



def parse_args(args=None):
  parser = argparse.ArgumentParser(description='Attempt to extract all attributes from pcl/impl/point_types.hpp.')
  parser.add_argument('path', help='The path pcl/impl/point_types.hpp.')
  parser.add_argument('-c', '--count', action='store_true', help='Count the number of PCL point types.')
  parser.add_argument('-l', '--list', action='store_true', help='List the PCL point types.')
  return parser.parse_args(args)



def parse_point_types(point_types_h):
  pcl_point_types_re = re.compile('#define\s+PCL_POINT_TYPES(.*?)\n\n', re.DOTALL)
  point_types_definition = pcl_point_types_re.search(point_types_h)
  if not point_types_definition:
    raise ValueError('failed to parse PCL_POINT_TYPES')
  point_types = point_types_definition.group(1).split('\\')
  for point_type in point_types:
    # [5:] to omit "pcl::"
    point_type = point_type.strip('() \n')[5:]
    if point_type:
      yield point_type



def parse_structs(point_types_h, point_types):
  if not isinstance(point_types, set):
    point_types = set(point_types)
  in_struct = False
  indent = ''
  for line in point_types_h.split('\n'):
    line = line.rstrip()
    sline = line.lstrip()
    if sline.startswith('struct ') and 'public' not in line:
      struct_name = sline.rsplit(None, 1)[-1].strip('_')
      indent = line[:len(line) - len(sline)]
      if struct_name in point_types:
        in_struct = True
    if in_struct:
      # Omit empty lines, friend declarations and descriptor size methods.
      if line and 'friend' not in line and 'descriptorSize ()' not in line:
        yield line
      if line == (indent + '};'):
        in_struct = False
        yield ''



def main(args=None):
  pargs = parse_args(args=args)
  with open(pargs.path, 'r') as f:
    point_types_h = f.read()

  try:
    point_types = tuple(parse_point_types(point_types_h))

    if pargs.count or pargs.list:
      n_pts = len(point_types)

      if pargs.count:
        print('number of PCL point types: {:d}'.format(n_pts))

      if pargs.list:
        fmt = '{{:{:d}d}} {{}}'.format(len(str(n_pts)))
        for i, pt in enumerate(point_types):
          print(fmt.format(i+1, pt))

    else:
      structs = '\n'.join(parse_structs(point_types_h, point_types))
      print(structs)

  except ValueError as e:
    e.msg += ' [{}]'.format(pargs.path)
    raise e



if __name__ == '__main__':
  try:
    main()
  except (KeyboardInterrupt, BrokenPipeError):
    pass
  except FileNotFoundError as e:
    import sys
    sys.exit(str(e))
