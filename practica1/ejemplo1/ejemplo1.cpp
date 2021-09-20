#include "ejemplo1.h"


ejemplo1::ejemplo1() : Ui_Counter() {
    setupUi(this);
    show();
    connect(button, SIGNAL(clicked()), this, SLOT(doButton()));
    connect(button_2, SIGNAL(clicked()), this, SLOT(doReset()));

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, QOverload<>::of(&ejemplo1::upd));
    timer->start(500);
}

ejemplo1::~ejemplo1() {

}

void ejemplo1::doButton() {
    stopped = !stopped;
    if (stopped) {
        timer->stop();
    } else
        timer->start(500);

    qDebug() << "click on stop";
}

void ejemplo1::doReset() {
    cont = -1;
    timer->start(500);
    stopped = false;

    qDebug() << "click on reset";
}

void ejemplo1::upd() {
    lcdNumber->display(++cont);

}


