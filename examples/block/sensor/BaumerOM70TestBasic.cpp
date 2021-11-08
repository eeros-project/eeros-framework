#include <modbus/modbus-tcp.h>  
#include <modbus/modbus.h>  
#include <iostream>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <bits/stdc++.h> 

using namespace std;

int main() {
  uint16_t tab_reg[64];
  
  // Create new tcp connection
  modbus_t* ctx = modbus_new_tcp("192.168.0.252", 502);
  printf("Modbus new TCP connection : 192.168.0.252 \n");
    
  // Test set slave
  modbus_set_slave(ctx, 1);
  auto slave_out = modbus_get_slave(ctx);
  printf("Modbus slave ID / unit identifier: %d \n", slave_out);
    
  // Connect
  auto connectOutput = modbus_connect(ctx);
  if (connectOutput == -1) {
    fprintf(stderr, "Modbus connection failed: %s\n", modbus_strerror(errno));
    modbus_free(ctx);
    return -1;
  } else printf("Modbus connection successfull: %d \n", connectOutput);

  // Read vendor information
  printf("Get vendor information: ");
  int addr = 0;
  int nb = 33;    
  int rc = modbus_read_input_registers(ctx, addr, nb, tab_reg);
  if (rc == -1) {
    printf("rc == -1 \n");
    fprintf(stderr, "%s\n", modbus_strerror(errno));
    return -1;
  }
  char vendorInfo[nb];
  for (int i = 0; i < rc; i++) {
    vendorInfo[i] = tab_reg[i];
    printf("%c", vendorInfo[i]);
  }
  printf("\n");
    
  // Read device info
  printf("Get device info: ");
  addr = 40;
  nb = 45;
  rc = modbus_read_input_registers(ctx, addr, nb, tab_reg);
  if (rc == -1) {
    printf("rc == -1 \n");
    fprintf(stderr, "%s\n", modbus_strerror(errno));
    return -1;
  }
  char deviceInfo[nb];
  for (int i = 0; i < rc; i++) {
    deviceInfo[i] = tab_reg[i];
    printf("%c", deviceInfo[i]);
  }
  printf("\n");
     
  // Read all measurements
  printf("Get all measurements: \n");
  addr = 200;
  nb = 17;
  
  time_t start = clock();
  ios_base::sync_with_stdio(false); 
  rc = modbus_read_input_registers(ctx, addr, nb, tab_reg);
  uint32_t* ptr_i = (uint32_t*)&tab_reg[3];
  std::cout << "i = 0x" << std::hex << *ptr_i << std::endl;
  float* ptr_f = reinterpret_cast<float*>(ptr_i);
  std::cout << "f = " << *ptr_f << std::endl;
  time_t end = clock(); 
    
  // Calculating total time taken by the program. 
  cout << " start " << start << endl;
  cout << " end " << end << endl;
  cout << " end-start " << (end-start) << endl;
  double time_taken = double(end - start) / double(CLOCKS_PER_SEC); 
  cout << "Time taken by program is : " << fixed  
         << time_taken << setprecision(5); 
  cout << " sec " << endl;  
        
  for(int i = 0; i < rc; i++) {
    printf("All Meas; reg[%d]=%d (0x%X)\n", i, tab_reg[i], tab_reg[i]);
  }

  modbus_close(ctx);
  modbus_free(ctx);
    
  return 0;
}

