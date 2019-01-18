#!/usr/bin/env python3

import argparse

INCLUDE_GUARD = 'PCLPHistogramPointTypes_h'
SEQUENCE_NAME = 'PCLP_HISTOGRAM_POINT_TYPES'
POINT_FMT = '(pcl::Histogram<{:d}>)'

def parse_args(args=None):
  parser = argparse.ArgumentParser(description='Generate a preprocessor sequence for n PCL histogram point types (pcl::Histogram<n>).')
  parser.add_argument('n', type=int, help='The highest supported value of n.')
  return parser.parse_args(args)

def main(args=None):
  pargs = parse_args(args=args)
  width = len(POINT_FMT.format(pargs.n))

  header = r'''// This file is generated automatically. Changes made to this file will be lost.
#ifndef {guard}
#define {guard}

#define {sequence_name} \
  {points}

#endif // {guard}'''.format(
    guard=INCLUDE_GUARD,
    sequence_name=SEQUENCE_NAME,
    points=' \\\n  '.join(POINT_FMT.format(i).ljust(width) for i in range(1, pargs.n+1))
  )
  print(header)

if __name__ == '__main__':
  try:
    main()
  except (KeyboardInterrupt, BrokenPipeError):
    pass

