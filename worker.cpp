#include "worker.h"
CloudWorker::CloudWorker(QObject* parent) : QObject(parent)
{
    connect(this, &CloudWorker::meshReady, this, &CloudWorker::finalProcessing);
}