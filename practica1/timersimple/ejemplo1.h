#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"
#include <chrono>
#include "timer.h"


class ejemplo1 : public QWidget, public Ui_Counter {
Q_OBJECT
public:
    ejemplo1();

    virtual ~ejemplo1();

public slots:

    //Manages the stop/start button.
    void doButton();

    //Resets the stopwatch by setting cont = 0.
    void doReset();

    //Changes the period.
    void doChangePeriod();

    //Returns the currently using period.
    void doCurrentPeriod();

    //Returns the elapsed time since the last time reset was pressed.
    void doTotalTime();

    //returns the elapsed time since the last time lap was pressed.
    void doLapTime();

private:
    Timer mytimer, mytimerLong;
    int cont = 0;
    bool stopped = false;
    int trick = 5;

    //Displays the stopwatch graphically
    void upd();

};

#endif // ejemplo1_H
