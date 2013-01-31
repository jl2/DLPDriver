
#include "dlpdriver.h"

DlpDriver *DlpDriver::singleton = 0;

DlpDriver * DlpDriver::GetDlpDriver() {
    if (DlpDriver::singleton == 0) {
        DlpDriver::singleton = new DlpDriver();
    }
    return DlpDriver::singleton;
}

DlpDriver::DlpDriver() {
}
