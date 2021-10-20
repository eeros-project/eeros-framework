#include <libmodbus-3.1.6/src/modbus-tcp.h>
#include <libmodbus-3.1.6/src/modbus.h>
#include <iostream>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <bits/stdc++.h> 

using namespace std;


modbus_t *ctx;
uint16_t tab_reg[64];
int rc;
int i;
int addr;
int nb;

int main() {
    time_t start, end; 

    // Create new tcp connection
    ctx = modbus_new_tcp("192.168.0.252", 502);
    printf("Modbus new TCP connection : 192.168.0.252 \n");
    
    // Test set slave
    modbus_set_slave(ctx, 1);
    auto slave_out = modbus_get_slave(ctx);
    printf("Modbus slave ID / unit identifier: %d \n", slave_out);
    
    // Connect
    auto connect_output = modbus_connect(ctx);
    
    
    if (connect_output == -1) {
        fprintf(stderr, "Modbus connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }
    else{
        printf("Modbus connection successfull: %d \n", connect_output);
    }

    
    // *** Input Register (FC04), Address 200, Size 17
    // ** Holding registers -> read holding registers FC03, write single holding register FC06, write multiple holding registers FC16
    // ** Input registers   -> read input registers FC04
    
    
    // *** Read vendor information
    printf("Get vendor information: ");
    addr = 0;
    nb = 33;    
    rc = modbus_read_input_registers(ctx, addr, nb, tab_reg);
    
    if (rc == -1) {
        printf("rc == -1 \n");
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        return -1;
    }
    char vendor_info[nb];
    for (i=0; i < rc; i++) {
        vendor_info[i] = tab_reg[i];
        printf("%c", vendor_info[i]);
    }
    printf("\n");

    
    // *** Read device info
    printf("Get device info: ");
    addr = 40;
    nb = 45;
    rc = modbus_read_input_registers(ctx, addr, nb, tab_reg);

    if (rc == -1) {
        printf("rc == -1 \n");
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        return -1;
    }
    char device_info[nb];
    for (i=0; i < rc; i++) {
        device_info[i] = tab_reg[i];
        printf("%c", device_info[i]);
    }
    printf("\n");

     
    // *** Read all measurements
    printf("Get all measurements: \n");
    addr = 200;
    nb = 17;
    
    start = clock();
    ios_base::sync_with_stdio(false); 
  
    rc = modbus_read_input_registers(ctx, addr, nb, tab_reg);
    
    // C++   
    uint32_t* ptr_i = (uint32_t*)&tab_reg[3];
    std::cout << "i = 0x" << std::hex << *ptr_i << std::endl;
    float* ptr_f = reinterpret_cast<float*>(ptr_i);
    std::cout << "f = " << *ptr_f << std::endl;
    
    end = clock(); 
    
    // Calculating total time taken by the program. 
    cout << " start " << start << endl;
    cout << " end " << end << endl;
    cout << " end-start " << (end-start) << endl;
    double time_taken = double(end - start) / double(CLOCKS_PER_SEC); 
    cout << "Time taken by program is : " << fixed  
         << time_taken << setprecision(5); 
    cout << " sec " << endl;  
        
    for (i=0; i < rc; i++) {
        printf("All Meas; reg[%d]=%d (0x%X)\n", i, tab_reg[i], tab_reg[i]);
    }
    
    
//     uint32_t distance = (tab_reg[4] << 16) + tab_reg[3];
//     std::cout << "i = 0x" << std::hex << distance << std::endl;
//     float* ptr_f2 = reinterpret_cast<float*>(&distance);
//     std::cout << "f = " << *ptr_f2 << std::endl;

    
    modbus_close(ctx);
    modbus_free(ctx);
    
    return 0;
}

