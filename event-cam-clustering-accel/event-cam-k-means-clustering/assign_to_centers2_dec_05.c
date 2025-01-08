#define _CRT_SECURE_NO_WARNINGS
#define PROGRAM_FILE "assign_to_centers.cl"

#define ARRAY_SIZE 2048
#define NUM_KERNELS 3

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef MAC
#include <OpenCL/cl.h>
#else
#include <CL/cl.h>
#endif

/* Find a GPU or CPU associated with the first available platform */
cl_device_id create_device()
{

   cl_platform_id platform;
   cl_device_id dev;
   int err;

   /* Identify a platform */
   err = clGetPlatformIDs(1, &platform, NULL);
   if (err < 0)
   {
      perror("Couldn't identify a platform");
      exit(1);
   }

   /* Access a device */
   err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &dev, NULL);
   if (err == CL_DEVICE_NOT_FOUND)
   {
      err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_CPU, 1, &dev, NULL);
   }
   if (err < 0)
   {
      perror("Couldn't access any devices");
      exit(1);
   }

   return dev;
}

/* Create program from a file and compile it */
cl_program build_program(cl_context ctx, cl_device_id dev, const char *filename)
{

   cl_program program;
   FILE *program_handle;
   char *program_buffer, *program_log;
   size_t program_size, log_size;
   int err;

   /* Read program file and place content into buffer */
   program_handle = fopen(filename, "r");
   if (program_handle == NULL)
   {
      perror("Couldn't find the program file");
      exit(1);
   }
   fseek(program_handle, 0, SEEK_END);
   program_size = ftell(program_handle);
   rewind(program_handle);
   program_buffer = (char *)malloc(program_size + 1);
   program_buffer[program_size] = '\0';
   fread(program_buffer, sizeof(char), program_size, program_handle);
   fclose(program_handle);

   /* Create program from file */
   program = clCreateProgramWithSource(ctx, 1,
                                       (const char **)&program_buffer, &program_size, &err);
   if (err < 0)
   {
      perror("Couldn't create the program");
      exit(1);
   }
   free(program_buffer);

   /* Build program */
   err = clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
   if (err < 0)
   {

      /* Find size of log and print to std output */
      clGetProgramBuildInfo(program, dev, CL_PROGRAM_BUILD_LOG,
                            0, NULL, &log_size);
      program_log = (char *)malloc(log_size + 1);
      program_log[log_size] = '\0';
      clGetProgramBuildInfo(program, dev, CL_PROGRAM_BUILD_LOG,
                            log_size + 1, program_log, NULL);
      printf("%s\n", program_log);
      free(program_log);
      exit(1);
   }

   return program;
}

int main()
{

   /* OpenCL structures */
   cl_device_id device;
   cl_context context;
   cl_program program;
   cl_kernel kernel;
   cl_command_queue queue;
   cl_event prof_event;
   cl_int i, j, err;
   size_t local_size, global_size;
   char kernel_names[NUM_KERNELS][20] =
       {"assign_to_centers", "assign_data_cluster", "reduction_scalar"};

   /* Data and buffers */
   float data[ARRAY_SIZE];
   printf("Data stored\n");
   for (int i=0; i<ARRAY_SIZE; i++)
   {
      int data_value = i % 100;
      data[i]=data_value;
      printf(" %d:%d, ", i, data_value);
   
   }
   printf("\n");
   float centroids[16] = {1, 1, 10, 10, 20, 20, 30, 30, 50, 50, 60, 60, 70, 70, 80, 80};
   float sum, actual_sum, *scalar_sum, *vector_sum;
   float output[ARRAY_SIZE*8];
   for (int i=0; i<ARRAY_SIZE*8; i++)
   {     
      output[i]=0;      
   }
   cl_int cluster_index[8] = {0,0,0,0,0,0,0,0};  // To track cluster last update point
   // unsigned int *assign;
   unsigned int assign[ARRAY_SIZE/2];
   cl_mem data_buffer, centroid_buffer, assign_buffer, scalar_sum_buffer, vector_sum_buffer, output_buffer;
   cl_int num_groups;
   cl_ulong time_start, time_end, total_time;

   /* Initialize data */
   // for(i=0; i<ARRAY_SIZE; i++) {
   //    data[i] = 1.0f*i;
   // }

   /* Create device and determine local size */
   device = create_device();
   err = clGetDeviceInfo(device, CL_DEVICE_MAX_WORK_GROUP_SIZE,
                         sizeof(local_size), &local_size, NULL);
   printf("local size : %d \n", local_size);
   local_size = 1024;
   if (err < 0)
   {
      perror("Couldn't obtain device information");
      exit(1);
   }

   /* Allocate and initialize output arrays */
   num_groups = ARRAY_SIZE / local_size;

   // assign = (unsigned int *)malloc(32 * sizeof(unsigned int));
   // scalar_sum = (float *)malloc(num_groups * sizeof(float));

   

   /* Create a context */
   context = clCreateContext(NULL, 1, &device, NULL, NULL, &err);
   if (err < 0)
   {
      perror("Couldn't create a context");
      exit(1);
   }

   /* Build program */
   program = build_program(context, device, PROGRAM_FILE);

   /* Create data buffer */
   data_buffer = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, ARRAY_SIZE * sizeof(float), data, &err);
   centroid_buffer = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, ARRAY_SIZE * sizeof(float), centroids, &err);

   // scalar_sum_buffer = clCreateBuffer(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, num_groups * sizeof(float), scalar_sum, &err);
   // vector_sum_buffer = clCreateBuffer(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, num_groups * sizeof(float), vector_sum, &err);
   assign_buffer = clCreateBuffer(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,  sizeof(assign), assign, &err);
   output_buffer = clCreateBuffer(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,  sizeof(output), output, &err);
   cl_mem cluster_index_buffer = clCreateBuffer(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, 
                                         sizeof(cluster_index), cluster_index, &err);
   
   
   if (err < 0)
   {
      perror("Couldn't create a buffer");
      exit(1);
   };

   /* Create a command queue */
   queue = clCreateCommandQueue(context, device,
                                CL_QUEUE_PROFILING_ENABLE, &err);
   if (err < 0)
   {
      perror("Couldn't create a command queue");
      exit(1);
   };

   /*************Start of cluster assignment kernel**************/

   /* Create a kernel */
   kernel = clCreateKernel(program, kernel_names[0], &err);
   if (err < 0)
   {
      perror("Couldn't create a kernel");
      exit(1);
   };

   /* Create kernel arguments */
   err = clSetKernelArg(kernel, 0, sizeof(cl_mem), &data_buffer);

   global_size = ARRAY_SIZE/2;
   // err |= clSetKernelArg(kernel, 1, local_size * sizeof(float), NULL);
   err |= clSetKernelArg(kernel, 1, sizeof(cl_mem), &centroid_buffer);
   err |= clSetKernelArg(kernel, 2, sizeof(cl_mem), &assign_buffer);

  


   if (err < 0)
   {
      perror("Couldn't create a kernel argument");
      exit(1);
   }

   /* Enqueue kernel */
   err = clEnqueueNDRangeKernel(queue, kernel, 1, NULL, &global_size,
                                &local_size, 0, NULL, &prof_event);
   if (err < 0)
   {
      perror("Couldn't enqueue the kernel");
      exit(1);
   }

   /* Finish processing the queue and get profiling information */
   clFinish(queue);
   clGetEventProfilingInfo(prof_event, CL_PROFILING_COMMAND_START,
                           sizeof(time_start), &time_start, NULL);
   clGetEventProfilingInfo(prof_event, CL_PROFILING_COMMAND_END,
                           sizeof(time_end), &time_end, NULL);
   total_time = time_end - time_start;

   /* Read the result */

    


   err = clEnqueueReadBuffer(queue, assign_buffer, CL_TRUE, 0,
                             sizeof(assign), assign, 0, NULL, NULL);
   
   
   if (err < 0)
   {
      perror("Couldn't read the buffer");
      exit(1);
   }

   
   // sum = 0.0f;
   // for (j = 0; j < num_groups; j++)
   // {
   //    sum += scalar_sum[j];
   // }
  

   /* Check result */
   printf("%s: \n", kernel_names[0]);

   printf("assigned centers: \n");
   for (int i = 0; i < ARRAY_SIZE/2; i++){
      // printf(" %d : %d, ", i, assign[i] );
       printf("  %d, ",  assign[i] );
   }
   printf("\n");
    
   // actual_sum = 1.0f * ARRAY_SIZE / 2 * (ARRAY_SIZE - 1);
   // if (fabs(sum - actual_sum) > 0.01 * fabs(sum))
   //    printf("Check failed.\n");
   // else
   //    printf("Check passed.\n");
   printf("Total time = %lu\n\n", total_time);

   /***********Start of the cluster data arrangement kernel**********/
    /* Create a kernel */
   kernel = clCreateKernel(program, kernel_names[1], &err);
   if (err < 0)
   {
      perror("Couldn't create a kernel");
      exit(1);
   };

   /* Create kernel arguments */
   err = clSetKernelArg(kernel, 0, sizeof(cl_mem), &data_buffer);

   global_size = ARRAY_SIZE/2;
   // err |= clSetKernelArg(kernel, 1, local_size * sizeof(float), NULL);
   err |= clSetKernelArg(kernel, 1, sizeof(cl_mem), &assign_buffer);
   err |= clSetKernelArg(kernel, 2, sizeof(cl_mem), &cluster_index_buffer);
   err |= clSetKernelArg(kernel, 3, sizeof(cl_mem), &output_buffer);

  


   if (err < 0)
   {
      perror("Couldn't create a kernel argument");
      exit(1);
   }

   /* Enqueue kernel */
   err = clEnqueueNDRangeKernel(queue, kernel, 1, NULL, &global_size,
                                &local_size, 0, NULL, &prof_event);
   if (err < 0)
   {
      perror("Couldn't enqueue the kernel");
      exit(1);
   }

   /* Finish processing the queue and get profiling information */
   clFinish(queue);
   clGetEventProfilingInfo(prof_event, CL_PROFILING_COMMAND_START,
                           sizeof(time_start), &time_start, NULL);
   clGetEventProfilingInfo(prof_event, CL_PROFILING_COMMAND_END,
                           sizeof(time_end), &time_end, NULL);
   total_time = time_end - time_start;

   /* Read the result */

    


   err = clEnqueueReadBuffer(queue, output_buffer, CL_TRUE, 0,
                             sizeof(output), output, 0, NULL, NULL);
   
   
   if (err < 0)
   {
      perror("Couldn't read the buffer");
      exit(1);
   }

   err = clEnqueueReadBuffer(queue, cluster_index_buffer, CL_TRUE, 0,
                             sizeof(cluster_index), cluster_index, 0, NULL, NULL);
   
   
   if (err < 0)
   {
      perror("Couldn't read the buffer");
      exit(1);
   }

   
   // sum = 0.0f;
   // for (j = 0; j < num_groups; j++)
   // {
   //    sum += scalar_sum[j];
   // }
  

   /* Check result */
   printf("%s: \n", kernel_names[1]);

   printf("updated cluster array: \n");
   for (int i = 0; i < 200; i++){
      printf("%f, ", output[i] );
   }
   printf("\n");
    
   // actual_sum = 1.0f * ARRAY_SIZE / 2 * (ARRAY_SIZE - 1);
   // if (fabs(sum - actual_sum) > 0.01 * fabs(sum))
   //    printf("Check failed.\n");
   // else
   //    printf("Check passed.\n");
   printf("Total time for second kernel = %lu\n\n", total_time);


   /********start of reduction kernel *******************/
   local_size = 1024;
   if (err < 0)
   {
      perror("Couldn't obtain device information");
      exit(1);
   }

   /* Allocate and initialize output arrays */
   num_groups = (ARRAY_SIZE*8) / local_size;

   printf("Number of groups : %d\n", num_groups);

   // assign = (unsigned int *)malloc(32 * sizeof(unsigned int));
   scalar_sum = (float *)malloc(num_groups * sizeof(float));

   scalar_sum_buffer = clCreateBuffer(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, num_groups * sizeof(float), scalar_sum, &err);

   
   for (i = 0; i < num_groups; i++)
   {
      scalar_sum[i] = 0.0f;
   }

    /* Create a kernel */
   kernel = clCreateKernel(program, kernel_names[2], &err);
   if (err < 0)
   {
      perror("Couldn't create a kernel");
      exit(1);
   };

     /* Create kernel arguments */
   err = clSetKernelArg(kernel, 0, sizeof(cl_mem), &output_buffer);

   global_size = ARRAY_SIZE*8;
   // err |= clSetKernelArg(kernel, 1, local_size * sizeof(float), NULL);
    err |= clSetKernelArg(kernel, 1, local_size * sizeof(float), NULL);
   err |= clSetKernelArg(kernel, 2, sizeof(cl_mem), &scalar_sum_buffer);

  


   if (err < 0)
   {
      perror("Couldn't create a kernel argument");
      exit(1);
   }

   /* Enqueue kernel */
   err = clEnqueueNDRangeKernel(queue, kernel, 1, NULL, &global_size,
                                &local_size, 0, NULL, &prof_event);
   if (err < 0)
   {
      perror("Couldn't enqueue the kernel");
      exit(1);
   }

   /* Finish processing the queue and get profiling information */
   clFinish(queue);
   clGetEventProfilingInfo(prof_event, CL_PROFILING_COMMAND_START,
                           sizeof(time_start), &time_start, NULL);
   clGetEventProfilingInfo(prof_event, CL_PROFILING_COMMAND_END,
                           sizeof(time_end), &time_end, NULL);
   total_time = time_end - time_start;

   /* Read the result */
   err = clEnqueueReadBuffer(queue, scalar_sum_buffer, CL_TRUE, 0, 
            num_groups * sizeof(float), scalar_sum, 0, NULL, NULL);     
   
   if (err < 0)
   {
      perror("Couldn't read the buffer");
      exit(1);
   }

   
   // sum = 0.0f;
   // for (j = 0; j < num_groups; j++)
   // {
   //    sum += scalar_sum[j];
   // }
   // printf("printing scalar sum : \n");
   // for (j = 0; j < num_groups; j++)
   // {
   //    printf("%f, ", scalar_sum[j]);
   // }
   // printf("\n");
  

   /* Check result */
   printf("%s: \n", kernel_names[2]);

   // printf("updated cluster array: \n");
   // for (int i = 4096; i < 4296; i++){
   //    printf("%f, ", output[i] );
   // }
   // printf("\n");
    
   // actual_sum = 1.0f * ARRAY_SIZE / 2 * (ARRAY_SIZE - 1);
   // if (fabs(sum - actual_sum) > 0.01 * fabs(sum))
   //    printf("Check failed.\n");
   // else
   //    printf("Check passed.\n");
   printf("Total time for third kernel = %lu\n\n", total_time);

   //Centroid calculation
   unsigned int CLUSTER_NUM=8;
   float new_centroids[CLUSTER_NUM*2];
   for (int i=0; i<CLUSTER_NUM*2; i+=2){
      new_centroids[i]=0;
      new_centroids[i+1]=0;
   }

   unsigned int y_offset = 2;

   for(int j=0; j<CLUSTER_NUM*2; j+=2){
      new_centroids[j] = (scalar_sum[j]+scalar_sum[j+1])/cluster_index[j/2];
      new_centroids[j+1] = (scalar_sum[j+y_offset]+scalar_sum[j+1+y_offset])/cluster_index[j/2];
   }
   
   printf("updated centroids\n");
   for (int i=0; i<CLUSTER_NUM*2; i+=2){
      printf("(%f, %f, %d) ",new_centroids[i],new_centroids[i+1], cluster_index[i/2]);
   }
   printf("\n");  


   /* Deallocate event */
   clReleaseEvent(prof_event);

   /* Deallocate resources */
   free(scalar_sum);
   free(vector_sum);
   // free(assign);

   clReleaseKernel(kernel);

   clReleaseMemObject(assign_buffer);
   clReleaseMemObject(centroid_buffer);
   clReleaseMemObject(data_buffer);
   clReleaseCommandQueue(queue);
   clReleaseProgram(program);
   clReleaseContext(context);
   return 0;
}
