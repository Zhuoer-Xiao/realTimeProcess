#pragma once
#pragma once
#include <QtWidgets/qinputdialog.h>
#include <QtWidgets/qformlayout.h>
#include <QtWidgets/QMainWindow>
#include "pclHeaders.h"
#include <vtkGenericOpenGLRenderWindow.h>
#include "ui_realTimeProcess.h"
#include <QMessageBox>

#include <QFileDialog>
#include "logger.h"
#include <boost/thread/thread.hpp>
#include "registrations.h"
#include "postProcess.h"
#include "dialogs.h"
#include "matcher.h"
#include <QSet>
#include <QThread>
// 多线程同步
#include <mutex>
#include <condition_variable>
#include <atomic>


class realTimeProcess : public QMainWindow {
	Q_OBJECT

public:
	realTimeProcess(QWidget* parent = nullptr);
	~realTimeProcess();

private:
	QThread m_workerThread;
	// 对话框
	QDialog* m_progressDialog = nullptr;
	vector<PointCloudT::Ptr> points;
	vector<PointCloudT::Ptr> pointsDown;
	mutex pointsMutex;
	condition_variable pointsCond;
	//完整点云
    PointCloudT::Ptr cloudShowDown;
	//结束标记
	std::atomic<bool> isRunning{false};
	int cloudNum = 0;
	mutex textMutex;

	Ui::realTimeProcessClass ui;

	PointCloudT::Ptr cloudShow;
	PointCloudT::Ptr cloudTar;
	PointCloudT::Ptr cloudSrc;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> view;
	//相关标志位
    bool isLoadSrc = false;
    bool isLoadTar = false;
	bool isReadDone= false;
	//编辑标记，0为源点云，1为目标点云
    int editFlag = 0;
	
	//匹配模块
	Matcher matcher;

	//日志
	Logger logger;

	void initialVtkWidget();
	//监控文件夹
	QString path;
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

	//更改当前编辑目标
	void editSrc();
	void editTar();

	//合并点云到目标点云
    void mergeCloud();

	//实时三维重建
	void realTimeSys();
	void folderMonitor();
	void stopSystem();
	void updateWhole();
	void showDialog();
	void PostProcessWhole();
signals:
	void updateVisualizationRequested();
};
