// OpenCL Kernel for processing 2D coordinate array with hash-based mapping
// Hash function to map 2D coordinates to a unique integer
int hash_coordinate(int x, int y) {
  // Ensure x and y are within expected ranges
  // int ix = convert_int_sat(x);
  // int iy = convert_int_sat(y);
  int width = 1280;
  int height = 720;

  // Use a suitable hash function that minimizes collisions
  // Prime number multiplication and bit mixing
  return (x * 1619 + y * 31) % 8192;
  // return (x ^ (y << 16)) % 4096;
}

__kernel void process_coordinates(
    __global const int
        *input_coords, // Flattened 2D coordinate array [x1, y1, x2, y2, ...]
    __global int *repeated_coords, // Output array for repeated coordinates
    __global int *unique_coords,   // Output array for unique coordinates
    __global int *repeated_count,  // Counter for repeated coordinates
    __global int *unique_count,    // Counter for unique coordinates
    const int total_coords,        // Total number of coordinates
    const int width,               // Width of image/coordinate space
    const int height)              // Height of image/coordinate space
{

  // Use local memory for tracking coordinate occurrences
  int MAX_HASH_SIZE = 8192;
  __local int coordinate_map[8192]; // Adjust MAX_HASH_SIZE based on your needs
  __local int local_repeated_count;
  __local int local_unique_count;

  // Initialize local counters
  if (get_local_id(0) == 0) {
    local_repeated_count = 0;
    local_unique_count = 0;
    // printf("local_unique_count: %d\n", local_unique_count);

    // Zero out the coordinate map
    for (int i = 0; i < 8192; i++) {
      coordinate_map[i] = 0;
    }
  }

  // Synchronize to ensure initialization
  barrier(CLK_LOCAL_MEM_FENCE);

  // Process coordinates
  for (int i = get_global_id(0); i < total_coords; i += get_global_size(0)) {
    // Extract x and y coordinates
    int x = input_coords[i * 2];
    int y = input_coords[i * 2 + 1];

    // Validate coordinate ranges
    if (x >= 0 && x <= 1280 && y >= 0 && y <= 720) {
      // Generate hash for the coordinate
      int hash = hash_coordinate(x, y);
      
      // printf("gid: %d Hash: %d, x:%d, y:%d\n", i, hash, x, y);
      // Atomic operations to track coordinate occurrences
      int prev_value = atomic_inc(&coordinate_map[hash]);

      // First occurrence
      if (prev_value == 0) {
        int index=atomic_inc(&local_unique_count);
        // printf("gid: %d, local_unique_count: %d, unique_count: %d\n", i, index, *unique_count);
        unique_coords[index*2] = x;
        unique_coords[index*2+1] = y;
        // printf("gid: %d Hash: %d, x:%f, y:%f\n", i, hash, x, y);
      }
      // Second occurrence (mark as repeated)
      else if (prev_value == 1) {
        atomic_inc(&local_repeated_count);
      }
      // printf("gid: %d Hash: %d, x:%f, y:%f, unique : %d, repeated: %d\n", i, hash, x, y, local_unique_count, local_repeated_count);
    }
  }

  // // Synchronize before writing results
  // barrier(CLK_LOCAL_MEM_FENCE);

  // // Write back results (first work-item does this)
  if (get_local_id(0) == 0) {
    atomic_add(repeated_count, local_repeated_count);
    atomic_add(unique_count, local_unique_count);
    printf("local_unique_count: %d, unique_count: %d\n", local_unique_count, *unique_count);
  }
}