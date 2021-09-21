#ifndef ejemplo1_H
#define ejemplo1_H


#include <QtGui>
#include "ui_counterDlg.h"
#include <QTimer>

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


private:
    QTimer *timer;
    int cont = 0;
    bool stopped = false;

    //Displays the stopwatch graphically
    void upd();

};

#endif // ejemplo1_H
