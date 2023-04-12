/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#include <unistd.h>
#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

namespace ORB_SLAM2
{

    System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
                   const bool bUseViewer) : mSensor(sensor), mpViewer(static_cast<Viewer *>(NULL)), mbReset(false), mbActivateLocalizationMode(false),
                                            mbDeactivateLocalizationMode(false)
    {
        // Output welcome message
        cout << endl
             << "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl
             << "This program comes with ABSOLUTELY NO WARRANTY;" << endl
             << "This is free software, and you are welcome to redistribute it" << endl
             << "under certain conditions. See LICENSE.txt." << endl
             << endl;

        cout << "Input sensor was set to: ";

        if (mSensor == MONOCULAR)
            cout << "Monocular" << endl;
        else if (mSensor == STEREO)
            cout << "Stereo" << endl;
        else if (mSensor == RGBD)
            cout << "RGB-D" << endl;

        // Check settings file
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "Failed to open settings file at: " << strSettingsFile << endl;
            exit(-1);
        }

        // Load ORB Vocabulary
        cout << endl
             << "Loading ORB Vocabulary. This could take a while..." << endl;

        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        if (!bVocLoad)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }
        cout << "Vocabulary loaded!" << endl
             << endl;

        // Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        // Create the Map
        mpMap = new Map();

        // Create Drawers. These are used by the Viewer
        mpFrameDrawer = new FrameDrawer(mpMap);
        mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

        // Initialize the Tracking thread
        //(it will live in the main thread of execution, the one that called this constructor)
        mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                                 mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

        // Initialize the Local Mapping thread and launch
        mpLocalMapper = new LocalMapping(mpMap, mSensor == MONOCULAR);
        mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

        // Initialize the Loop Closing thread and launch
        mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor != MONOCULAR);
        mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

        // Initialize the Viewer thread and launch
        if (bUseViewer)
        {
            mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile);
            mptViewer = new thread(&Viewer::Run, mpViewer);
            mpTracker->SetViewer(mpViewer);
        }

        // Set pointers between threads
        mpTracker->SetLocalMapper(mpLocalMapper);
        mpTracker->SetLoopClosing(mpLoopCloser);

        mpLocalMapper->SetTracker(mpTracker);
        mpLocalMapper->SetLoopCloser(mpLoopCloser);

        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);
    }

    cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
    {
        if (mSensor != STEREO)
        {
            cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
            exit(-1);
        }

        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode)
            {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped())
                {
                    usleep(1000);
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode)
            {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset)
            {
                mpTracker->Reset();
                mbReset = false;
            }
        }

        cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft, imRight, timestamp);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
        return Tcw;
    }

    cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
    {
        if (mSensor != RGBD)
        {
            cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
            exit(-1);
        }

        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode)
            {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped())
                {
                    usleep(1000);
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode)
            {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset)
            {
                mpTracker->Reset();
                mbReset = false;
            }
        }

        cv::Mat Tcw = mpTracker->GrabImageRGBD(im, depthmap, timestamp);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
        return Tcw;
    }

    cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
    {
        if (mSensor != MONOCULAR)
        {
            cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
            exit(-1);
        }

        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode)
            {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped())
                {
                    usleep(1000);
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode)
            {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset)
            {
                mpTracker->Reset();
                mbReset = false;
            }
        }

        cv::Mat Tcw = mpTracker->GrabImageMonocular(im, timestamp);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

        return Tcw;
    }

    void System::ActivateLocalizationMode()
    {
        unique_lock<mutex> lock(mMutexMode);
        mbActivateLocalizationMode = true;
    }

    void System::DeactivateLocalizationMode()
    {
        unique_lock<mutex> lock(mMutexMode);
        mbDeactivateLocalizationMode = true;
    }

    bool System::MapChanged()
    {
        static int n = 0;
        int curn = mpMap->GetLastBigChangeIdx();
        if (n < curn)
        {
            n = curn;
            return true;
        }
        else
            return false;
    }

    void System::Reset()
    {
        unique_lock<mutex> lock(mMutexReset);
        mbReset = true;
    }

    void System::Shutdown()
    {
        mpLocalMapper->RequestFinish();
        mpLoopCloser->RequestFinish();
        if (mpViewer)
        {
            mpViewer->RequestFinish();
            while (!mpViewer->isFinished())
                usleep(5000);
        }

        // Wait until all thread have effectively stopped
        while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
        {
            usleep(5000);
        }

        if (mpViewer)
            pangolin::BindToContext("ORB-SLAM2: Map Viewer");
    }

    void System::SaveTrajectoryTUM(const string &filename)
    {
        cout << endl
             << "Saving camera trajectory to " << filename << " ..." << endl;
        if (mSensor == MONOCULAR)
        {
            cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
            return;
        }

        vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        cv::Mat Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<ORB_SLAM2::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();
        for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                                     lend = mpTracker->mlRelativeFramePoses.end();
             lit != lend; lit++, lRit++, lT++, lbL++)
        {
            if (*lbL)
                continue;

            KeyFrame *pKF = *lRit;

            cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            while (pKF->isBad())
            {
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
            }

            Trw = Trw * pKF->GetPose() * Two;

            cv::Mat Tcw = (*lit) * Trw;
            cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
            cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

            vector<float> q = Converter::toQuaternion(Rwc);

            // f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
            f << setprecision(6) << " " << setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }
        f.close();
        cout << endl
             << "trajectory saved!" << endl;
    }

    void System::SaveKeyFrameTrajectoryTUM(const string &filename)
    {
        cout << endl
             << "Saving keyframe trajectory to " << filename << " ..." << endl;

        vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        // cv::Mat Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];

            // pKF->SetPose(pKF->GetPose()*Two);

            if (pKF->isBad())
                continue;

            cv::Mat R = pKF->GetRotation().t();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat t = pKF->GetCameraCenter();
            f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
              << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
            /* f << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
              << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl; */
        }

        f.close();
        cout << endl
             << "trajectory saved!" << endl;

        ofstream ff(filename, ios::app);
        for (auto &x : vpKFs)
        {
            ff << x->mTcwGBA << endl;
        }
        ff.close();
        cout << endl
             << "TcwBefGBA is ok!" << endl;
    }

    void System::SaveTrajectoryKITTI(const string &filename)
    {
        cout << endl
             << "Saving camera trajectory to " << filename << " ..." << endl;
        if (mSensor == MONOCULAR)
        {
            cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
            return;
        }

        vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        cv::Mat Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<ORB_SLAM2::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(), lend = mpTracker->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++)
        {
            ORB_SLAM2::KeyFrame *pKF = *lRit;

            cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

            while (pKF->isBad())
            {
                //  cout << "bad parent" << endl;
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
            }

            Trw = Trw * pKF->GetPose() * Two;

            cv::Mat Tcw = (*lit) * Trw;
            cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
            cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

            f << setprecision(9) << Rwc.at<float>(0, 0) << " " << Rwc.at<float>(0, 1) << " " << Rwc.at<float>(0, 2) << " " << twc.at<float>(0) << " " << Rwc.at<float>(1, 0) << " " << Rwc.at<float>(1, 1) << " " << Rwc.at<float>(1, 2) << " " << twc.at<float>(1) << " " << Rwc.at<float>(2, 0) << " " << Rwc.at<float>(2, 1) << " " << Rwc.at<float>(2, 2) << " " << twc.at<float>(2) << endl;
        }
        f.close();
        cout << endl
             << "trajectory saved!" << endl;
    }

    int System::GetTrackingState()
    {
        unique_lock<mutex> lock(mMutexState);
        return mTrackingState;
    }

    vector<MapPoint *> System::GetTrackedMapPoints()
    {
        unique_lock<mutex> lock(mMutexState);
        return mTrackedMapPoints;
    }

    vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
    {
        unique_lock<mutex> lock(mMutexState);
        return mTrackedKeyPointsUn;
    }

    void System::getlength()
    {
        std::vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        cout << "GetAllKeyFrame(): " << vpKFs.size() << endl;

        KeyFrame *kf0 = vpKFs[0];
        KeyFrame *kf1 = vpKFs[1];
        KeyFrame *kf2 = vpKFs[2];
        cout << "GetVectorCovisibleKeyFrame()0: " << kf0->GetVectorCovisibleKeyFrames().size() << endl;
        cout << "GetVectorCovisibleKeyFrame()1: " << kf1->GetVectorCovisibleKeyFrames().size() << endl;
        cout << "GetVectorCovisibleKeyFrame()2: " << kf2->GetVectorCovisibleKeyFrames().size() << endl;
        cout << "----------" << endl;

        cout << "kf0covkf1: " << kf0->GetWeight(kf1) << endl;
        cout << "kf0covkf2: " << kf0->GetWeight(kf2) << endl;
        cout << "kf1covkf2: " << kf1->GetWeight(kf2) << endl;
        cout << "kf2covkf1: " << kf2->GetWeight(kf1) << endl;

        cout << "nextId0: " << kf0->nNextId << endl;
        cout << "nextId1: " << kf1->nNextId << endl;
        cout << "nextId2: " << kf2->nNextId << endl;
        cout << "mnFrameId0: " << kf0->mnFrameId << endl;
        cout << "mnFrameId1: " << kf1->mnFrameId << endl;
        cout << "mnFrameId2: " << kf2->mnFrameId << endl;
        cout << "mnId0: " << kf0->mnId << endl;
        cout << "mnId1: " << kf1->mnId << endl;
        cout << "mnId2: " << kf2->mnId << endl;

        cout << "----------" << endl;
        cout << "kf0->GetChilds().size(): " << kf0->GetChilds().size() << endl;
        cout << "kf1->GetChilds().size(): " << kf1->GetChilds().size() << endl;
        cout << "kf2->GetChilds().size(): " << kf2->GetChilds().size() << endl;
    }

    std::vector<pair<int, vector<pair<int, int>>>> System::get_essgraph()
    {
        std::vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
        vector<pair<int, int>> cur;

        for (size_t i = 0; i < vpKFs.size(); ++i)
        {
            if (vpKFs[i]->isBad())
                continue;
            for (auto x : vpKFs)
            {
                if (x->isBad() || x == vpKFs[i] || vpKFs[i]->GetWeight(x) < 100)
                    continue;
                cur.push_back({x->mnFrameId, vpKFs[i]->GetWeight(x)});
            }
            essgraph.push_back({vpKFs[i]->mnFrameId, cur});
            cur.clear();
        }
        return essgraph;
    }

    std::vector<pair<int, vector<pair<int, int>>>> System::get_cgraph()
    {
        // cgraph存储格式为：vector<pair<本关键帧id, vector<pair<与本关键帧共视的关键帧id, 二者权重>>>>
        std::vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
        vector<pair<int, int>> cur;

        for (size_t i = 0; i < vpKFs.size(); ++i)
        {
            if (vpKFs[i]->isBad())
                continue;
            for (auto x : vpKFs)
            {
                if (x->isBad() || x == vpKFs[i])
                    continue;
                cur.push_back({x->mnFrameId, vpKFs[i]->GetWeight(x)});
            }
            cgraph.push_back({vpKFs[i]->mnFrameId, cur});
            cur.clear();
        }
        return cgraph;
    }

    void System::SaveCovGraph(const string &filename)
    {
        cout << endl
             << "Saving cov graph to " << filename << '...' << endl;
        std::vector<pair<int, vector<pair<int, int>>>> cg = get_cgraph();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (size_t i = 0; i < cg.size(); ++i)
        {
            f << "[KF_id " << cg[i].first << "]: ";
            for (size_t j = 0; j < cg[i].second.size(); ++j)
            {
                f << "{" << cg[i].second[j].first << ", " << cg[i].second[j].second << "} ";
            }
            f << endl;
        }
        f.close();
        cout << endl
             << "cov graph saved!" << endl;
    }

    std::vector<int> System::get_vkfid()
    {
        vector<int> ans;
        vector<ORB_SLAM2::KeyFrame *> vkfs = mpMap->GetAllKeyFrames();
        for (auto &x : vkfs)
        {
            ans.push_back(x->mnFrameId);
        }
        return ans;
    }

    void System::Save_node_id(const string &filename)
    {
        cout << endl
             << "Saving kf node to " << filename << "..." << endl;
        std::vector<int> kfsid = get_vkfid();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        f << '[';
        for (size_t i = 0; i < kfsid.size(); ++i)
        {
            if (i == kfsid.size() - 1)
                f << kfsid[i];
            else
                f << kfsid[i] << ", ";
        }
        f << ']';
        f.close();

        cout << endl
             << "kf node saved!" << endl;
    }

    void System::SaveEss(const string &filename)
    {
        cout << endl
             << "Saving ess graph to " << filename << "..." << endl;
        std::vector<pair<int, vector<pair<int, int>>>> eg = get_cgraph();
        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (size_t i = 0; i < eg.size(); ++i)
        {
            for (size_t j = 0; j < eg[i].second.size(); ++j)
            {
                if (i == eg.size() - 1 && j == eg[i].second.size() - 1)
                    f << '(' << eg[i].first << ", " << eg[i].second[j].first << ", " << eg[i].second[j].second << ")";
                else
                    f << '(' << eg[i].first << ", " << eg[i].second[j].first << ", " << eg[i].second[j].second << "), ";
            }
        }

        f.close();
        cout << endl
             << "ess graph saved!" << endl;
    }

    std::vector<pair<int, vector<pair<int, int>>>> System::get_spanningtree()
    {
        vector<KeyFrame *> vkfs = mpMap->GetAllKeyFrames();
        sort(vkfs.begin(), vkfs.end(), KeyFrame::lId);
        vector<pair<int, int>> cur;

        for (size_t i = 0; i < vkfs.size(); ++i)
        {
            if (vkfs[i]->isBad())
                continue;
            for (auto x : vkfs[i]->GetChilds())
            {
                if (!x->isBad())
                    cur.push_back({x->mnFrameId, vkfs[i]->GetWeight(x)});
            }
            spanningtree.push_back({vkfs[i]->mnFrameId, cur});
            cur.clear();
        }
        return spanningtree;
    }

    void System::SaveSpanningtree(const string &filename)
    {
        cout << endl
             << "Saving spanningtree to " << filename << "..." << endl;
        std::vector<pair<int, vector<pair<int, int>>>> st = get_spanningtree();
        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (size_t i = 0; i < st.size(); ++i)
        {
            for (size_t j = 0; j < st[i].second.size(); ++j)
            {
                if (i == st.size() - 1 && j == st[i].second.size() - 1)
                    f << '(' << st[i].first << ", " << st[i].second[j].first << ", " << st[i].second[j].second << ")";
                else
                    f << '(' << st[i].first << ", " << st[i].second[j].first << ", " << st[i].second[j].second << "), ";
            }
        }

        f.close();
        cout << endl
             << "spanningtree saved!" << endl;
    }

    void System::SaveVertex(const string &filename)
    {
        /* cout << endl
             << "Saving pose graph vertex to " << filename << "..." << endl; */

        vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        // cv::Mat Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];

            // pKF->SetPose(pKF->GetPose()*Two);

            if (pKF->isBad())
                continue;

            cv::Mat R = pKF->GetRotation().t();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat t = pKF->GetCameraCenter();
            f << "VERTEX_SE3:QUAT " << pKF->mnId << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
              << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
            /* f << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
              << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl; */
        }

        f.close();
        /* cout << endl
             << "vertex saved!" << endl; */
    }

    void System::SaveEdge(const string &filename)
    {
        /* cout << endl
             << "Saving pose graph edges to " << filename << "..." << endl; */
        vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        unordered_map<int, KeyFrame *> ump;
        for (auto &x : vpKFs)
            ump[x->mnId] = x;
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
        ofstream f;
        f.open(filename.c_str(), ios::app);
        f << fixed;

        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];
            if (pKF->isBad())
                continue;
            // 搜索这个关键帧的所有Tij
            for (unordered_map<int, cv::Mat>::iterator x = pKF->mumpTij.begin(); x != pKF->mumpTij.end(); ++x)
            {
                if (ump.count(x->first) == 0)
                    continue;
                cv::Mat Tij = x->second;
                cv::Mat R = pKF->GetR(Tij).t();
                vector<float> q = Converter::toQuaternion(R);
                cv::Mat t = pKF->GetT(Tij);
                f << "EDGE_SE3:QUAT " << pKF->mnId << " " << x->first << " " << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
                  << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " 10000 0 0 0 0 0 10000 0 0 0 0 10000 0 0 0 40000 0 0 40000 0 40000" << endl;
            }
            /* cout << endl
                 << "edges saved!" << endl; */
        }
    }

    void System::SaveEdge_no_error(const string &filename)
    {
        /* cout << endl
             << "Saving pose graph edges without error to " << filename << "..." << endl; */
        vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();

        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
        ofstream f;
        f.open(filename.c_str(), ios::app);
        f << fixed;

        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];
            if (pKF->isBad())
                continue;
            for (size_t j = i + 1; j < vpKFs.size(); j++)
            {
                if (vpKFs[j]->isBad())
                    continue;
                cv::Mat Tij = pKF->GetPose() * vpKFs[j]->GetPoseInverse();
                cv::Mat R = pKF->GetR(Tij).t();
                vector<float> q = Converter::toQuaternion(R);
                cv::Mat t = pKF->GetT(Tij);
                f << "EDGE_SE3:QUAT " << pKF->mnId << " " << vpKFs[j]->mnId << " " << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
                  << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " 10000 0 0 0 0 0 10000 0 0 0 0 10000 0 0 0 40000 0 0 40000 0 40000" << endl;
            }
        }
    }

    void System::SavePoseGraph(const string &filename)
    {
        cout << endl
             << "Saving pose graph to " << filename << "..." << endl;
        SaveVertex(filename);
        SaveEdge(filename);
        cout << endl
             << "pose graph saved!" << endl;
    }

    void System::SavePoseGraph_no_error(const string &filename)
    {
        cout << endl
             << "Saving pose graph without error to " << filename << "..." << endl;
        SaveVertex(filename);
        SaveEdge_no_error(filename);
        cout << endl
             << "pose graph without error saved!" << endl;
    }
} // namespace ORB_SLAM
