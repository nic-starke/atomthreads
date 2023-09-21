project('atomthreads', 'c')

atomthreads_sources = files(
  'kernel/atomkernel.c',
  'kernel/atommutex.c',
  'kernel/atomqueue.c',
  'kernel/atomsem.c',
  'kernel/atomtimer.c',
  'ports/avr/atomport-private.c',
  'ports/avr/atomport.c',
  'ports/avr/atomport-asm.s'
)

add_project_arguments('-DATOM_STACK_CHECKING', language : 'c')

atomthreads_inc = include_directories('./kernel', './ports/avr', './tests')
atomthreads_lib = library('atomthreads', atomthreads_sources, include_directories: atomthreads_inc)
atomthreads_dep = declare_dependency(link_with : atomthreads_lib, include_directories : atomthreads_inc)