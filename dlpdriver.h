#ifndef DLP_2232_PB_H
#define DLP_2232_PB_H

class DlpDriver {
public:
    static DlpDriver * GetDlpDriver();

private:
    DlpDriver();

    DlpDriver operator=(DlpDriver&);
    DlpDriver(DlpDriver&);

    static DlpDriver *singleton;
};

#endif
