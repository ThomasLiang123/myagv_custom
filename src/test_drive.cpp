#include "myagv_custom/movement.h"

int main(int argc, char* argv[]) {
    init_myagv(argc, argv);
    
    move_myagv(0.3, 0, 10, 4);
    //spin_myagv(30, 8);
    move_myagv(-0.3, 0, 10, 4);
    
    return 0;
}