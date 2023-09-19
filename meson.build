project('atomthreads', 'c')

atomthreads_sources = files(
  'kernel/atomkernel.c',
  'kernel/atommutex.c',
  'kernel/atomqueue.c',
  'kernel/atomsem.c',
  'kernel/atomtimer.c',
)

atomthreads_inc = include_directories('.')

atomthreads = library('atomthreads', atomthreads_sources, include_directories: atomthreads_inc)