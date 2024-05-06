// ipmmpc_interface.cpp

#include "ipmmpc_interface.h"
#include "ipmMPC.hpp"

void* create_ipmmpc() {
    return new IPMMPC();
}

void destroy_ipmmpc(void* ipmmpc) {
    delete static_cast<IPMMPC*>(ipmmpc);
}

// Implement other functions for calling methods on IPMMPC