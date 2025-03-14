#pragma once
#pragma once

#include <QtWidgets/QMainWindow>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include "ui_realTimeProcess.h"
#include <QMessageBox>
#include "ui_realSystem.h"

class realTimeProcess : public QMainWindow {
	Q_OBJECT

public:
	realTimeProcess(QWidget* parent = nullptr);
	~realTimeProcess();

private:
	Ui::realTimeProcessClass ui;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> view;

	void initialVtkWidget();

private slots:
	void onOpen();
	void enterSystem();
};

