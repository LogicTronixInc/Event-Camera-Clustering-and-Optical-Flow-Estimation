__kernel void assign_to_centers(__global float* data, 
      __global float *centers, __global int *assignments) {

   int lid = get_local_id(0);
   int group_size = get_local_size(0);
   uint gx = get_global_id(0)*2;
   
   float data_x = data[gx];
   float data_y = data[gx+1];

   float threshold_dd = 50.0;
   uchar indMin = -1;
    
   for (int i = 0; i < 16; i+=2) {
       float localDx = (centers[i] - data_x);
       float localDy = (centers[i+1] - data_y);
       float3 vec = (float3)(localDx, localDy, 0);
       float localD = length(vec);
       
       //printf("%f, %f, %f\n", localDx, localDy, localD);
	   if (localD < threshold_dd) {
		   indMin = i;		
         //assignments[lid]  = i; 
         threshold_dd = localD;
	   }
      assignments[gx/2]  = indMin;
      // printf("%d, %d, %d : %f, %f, %f, %f : %f, %f, %f - %d, %f\n", lid, gx, i, data_x, data_y, centers[i], centers[i+1], localDx, localDy, localD, assignments[gx/2], threshold_dd );
      
   }
   
   


}

__kernel void assign_data_cluster(__global float* data, 
      __global unsigned int *assign, __global atomic_int *cluster_index, __global float* output) {

   int lid = get_local_id(0);
   int group_size = get_local_size(0);

   uint gid = get_global_id(0)*2;
   uint assign_cluster = assign[gid/2]/2;

   uint address_offset = assign_cluster * 4096 ;
   uint address_offset_y = address_offset + 2048;

   if (assign_cluster == 0){
      int index0 = atomic_fetch_add(&cluster_index[0],1);
      output[address_offset+index0]=data[gid]; 
      output[address_offset_y+index0] = data[gid+1];
      // printf("gid: %d, address_offset: %d, cluster id:%d, index : %d, data: %f \t", gid, address_offset, assign_cluster, index0, data[gid]);   
      // printf("gid: %d, address_offset_y: %d, cluster id:%d, index : %d, data: %f \t", gid, address_offset_y, assign_cluster, index0, data[gid+1]);    

   }

   if (assign_cluster == 1){
      int index1 = atomic_fetch_add(&cluster_index[1],1);
      output[address_offset+index1]= data[gid]; 
      output[address_offset_y+index1] = data[gid+1];
      //printf("address_offset: %d, index : %d, data: %f \t", address_offset, index1, data[gid]);

   }

   if (assign_cluster == 2){
      int index2 = atomic_fetch_add(&cluster_index[2],1);
      output[address_offset+index2]= data[gid]; 
      output[address_offset_y+index2] = data[gid+1];
      //printf("address_offset: %d, index : %d, data: %f \t", address_offset, index2, data[gid]);

   }

   if (assign_cluster == 3){
      int index3 = atomic_fetch_add(&cluster_index[3],1);
      output[address_offset+index3]= data[gid]; 
      output[address_offset_y+index3] = data[gid+1];
      //printf("address_offset: %d, index : %d, data: %f \t", address_offset, index3, data[gid]);

   }

   if (assign_cluster == 4){
      int index4 = atomic_fetch_add(&cluster_index[4],1);
      output[address_offset+index4]= data[gid]; 
      output[address_offset_y+index4] = data[gid+1];
      //printf("address_offset: %d, index : %d, data: %f \t", address_offset, index4, data[gid]);

   }

   if (assign_cluster == 5){
      int index5 = atomic_fetch_add(&cluster_index[5],1);
      output[address_offset+index5]= data[gid]; 
      output[address_offset_y+index5] = data[gid+1];
      //printf("address_offset: %d, index : %d, data: %f \t", address_offset, index5, data[gid]);

   }

   if (assign_cluster == 6){
      int index6 = atomic_fetch_add(&cluster_index[6],1);
      output[address_offset+index6]= data[gid]; 
      output[address_offset_y+index6] = data[gid+1];
      //printf("address_offset: %d, index : %d, data: %f \t", address_offset, index6, data[gid]);

   }

   if (assign_cluster == 7){
      int index7 = atomic_fetch_add(&cluster_index[7],1);
      output[address_offset+index7]= data[gid]; 
      output[address_offset_y+index7] = data[gid+1];
      //printf("address_offset: %d, index : %d, data: %f \t", address_offset, index7, data[gid]);

   }



   
   //printf("group id %d: value %d ", gid, output[gid]);

  
}

__kernel void reduction_scalar(__global float* data, 
      __local float* partial_sums, __global float* output) {

   int lid = get_local_id(0);
   int group_size = get_local_size(0);

   partial_sums[lid] = data[get_global_id(0)];
   barrier(CLK_LOCAL_MEM_FENCE);

   for(int i = group_size/2; i>0; i >>= 1) {
      if(lid < i) {
         partial_sums[lid] += partial_sums[lid + i];
      }
      barrier(CLK_LOCAL_MEM_FENCE);
   }

   if(lid == 0) {
      output[get_group_id(0)] = partial_sums[0];
   }
}

