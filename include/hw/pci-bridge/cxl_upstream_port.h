
#ifndef CXL_USP_H
#define CXL_USP_H
#include "hw/pci/pcie.h"
#include "hw/pci/pcie_port.h"
#include "hw/cxl/cxl.h"

typedef struct CXLUpstreamPort {
    /*< private >*/
    PCIEPort parent_obj;

    /*< public >*/
    CXLComponentState cxl_cstate;
    CXLCCI swcci;

    PCIExpLinkSpeed speed;
    PCIExpLinkWidth width;

    DOECap doe_cdat;
    uint64_t sn;
    char *vcs_name;
    uint8_t ppb;
} CXLUpstreamPort;

#endif /* CXL_SUP_H */
