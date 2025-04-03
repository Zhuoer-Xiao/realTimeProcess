#pragma once
#pragma once
#include <QtWidgets/qinputdialog.h>
#include <QtWidgets/qformlayout.h>
#include <QtWidgets/QMainWindow>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>s
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include "ui_realTimeProcess.h"
#include <QMessageBox>
#include <pcl/visualization/pcl_visualizer.h>
#include <QFileDialog>
#include "realSystem.h"
#include "logger.h"
#include <boost/thread/thread.hpp>
#include "registrations.h"
#include "postProcess.h"
#include "dialogs.h"
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class realTimeProcess : public QMainWindow {
	Q_OBJECT

public:
	realTimeProcess(QWidget* parent = nullptr);
	~realTimeProcess();

private:
	Ui::realTimeProcessClass ui;

	PointCloudT::Ptr cloudShow;
	PointCloudT::Ptr cloudTar;
	PointCloudT::Ptr cloudSrc;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> view;
	//相关标志位
    bool isLoadSrc = false;
    bool isLoadTar = false;
	//编辑标记，0为源点云，1为目标点云
    int editFlag = 0;


	//日志
	Logger logger;

	void initialVtkWidget();

private slots:
	//todo:加载点云文件
	void loadSrc();
	void loadTar();
	//todo:进入实时拼接系统
	void enterSystem();
	//todo:开始拼接
	void startReg();
	//todo:开始后处理
	void startPostProcess();
	//todo:清除点云
	void clearPointCloud();
	//todo:背景颜色设定
    void setBackgroundColor();
	//todo:保存当前点云
	void saveCurrentPointCloud();
	//todo:errorInfo
    void errorInfo(int errNum);
	//todo:更新显示点云
	void updatePointCloudShow();

	//
	void editSrc();
	void editTar();

	//合并点云到目标点云
    void mergeCloud();
};

