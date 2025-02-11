////////////////////////////////////////////////////////////////////////////////
//  VSLAMLogger.cpp
//
//  This is the implementation of VSLAMLogger class.
//
//  Copyright 2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef _MSC_VER
#pragma warning(disable : 4244)
#endif

#include <VSLAMLogger.hpp>

namespace vision {
    namespace vslam {

        VSLAMLogger::VSLAMLogger(const int _verboseLevel) : VSLAMLoggerBase(_verboseLevel) {
            this->initializeCatalogMap();
        }

        VSLAMLogger::VSLAMLogger(const int _verboseLevel, const std::string& _filename) 
            : VSLAMLoggerBase(_verboseLevel, _filename) {
            this->initializeCatalogMap();
        }

        void VSLAMLogger::initializeCatalogMap() {

            //
            // Initialization
            //
            auto umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::systemSetup());
            this->catalogMap["vision::vslamVerboseCpp::systemSetup"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::systemSetupRGBD());
            this->catalogMap["vision::vslamVerboseCpp::systemSetupRGBD"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::systemSetupStereo());
            this->catalogMap["vision::vslamVerboseCpp::systemSetupStereo"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::startMapInitialization());
            this->catalogMap["vision::vslamVerboseCpp::startMapInitialization"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::failedMapInitialization("%s", "%s"));
            this->catalogMap["vision::vslamVerboseCpp::failedMapInitialization"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::failedMapInitializationOneFrame("%s"));
            this->catalogMap["vision::vslamVerboseCpp::failedMapInitializationOneFrame"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::endMapInitialization("%s", "%s"));
            this->catalogMap["vision::vslamVerboseCpp::endMapInitialization"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::endMapInitializationOneFrame("%s"));
            this->catalogMap["vision::vslamVerboseCpp::endMapInitializationOneFrame"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::MatchedPointsNotEnough("%s", "%s", "%s"));
            this->catalogMap["vision::vslamVerboseCpp::MatchedPointsNotEnough"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::ExtractedPointsNotEnough("%s", "%s", "%s"));
            this->catalogMap["vision::vslamVerboseCpp::ExtractedPointsNotEnough"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::ReprojectionErrorH("%s"));
            this->catalogMap["vision::vslamVerboseCpp::ReprojectionErrorH"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::ReprojectionErrorF("%s"));
            this->catalogMap["vision::vslamVerboseCpp::ReprojectionErrorF"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::UsingHMatrix());
            this->catalogMap["vision::vslamVerboseCpp::UsingHMatrix"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::UsingFMatrix());
            this->catalogMap["vision::vslamVerboseCpp::UsingFMatrix"] = std::string(umsg.begin(), umsg.end());

            //
            // Tracking
            //
            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::startTracking("%s"));
            this->catalogMap["vision::vslamVerboseCpp::startTracking"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::keyFrameDetected("%s"));
            this->catalogMap["vision::vslamVerboseCpp::keyFrameDetected"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::keyFrameDetectedWeakFrame());
            this->catalogMap["vision::vslamVerboseCpp::keyFrameDetectedWeakFrame"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::keyFrameDetectedManyNonKeyFrames());
            this->catalogMap["vision::vslamVerboseCpp::keyFrameDetectedManyNonKeyFrames"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::PoseBeforeMotionBA());
            this->catalogMap["vision::vslamVerboseCpp::PoseBeforeMotionBA"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::PoseAfterMotionBA());
            this->catalogMap["vision::vslamVerboseCpp::PoseAfterMotionBA"] = std::string(umsg.begin(), umsg.end());
 
            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::localKeyFramesAndPoints("%s"));
            this->catalogMap["vision::vslamVerboseCpp::localKeyFramesAndPoints"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::trackingLostAtFrame());
            this->catalogMap["vision::vslamVerboseCpp::trackingLostAtFrame"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::NumTrackedPointsWithThreshold("%s", "%s"));
            this->catalogMap["vision::vslamVerboseCpp::NumTrackedPointsWithThreshold"] = std::string(umsg.begin(), umsg.end());
 
            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::NumInlierPointsWithThreshold("%s", "%s"));
            this->catalogMap["vision::vslamVerboseCpp::NumInlierPointsWithThreshold"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::MotionBAStart());
            this->catalogMap["vision::vslamVerboseCpp::MotionBAStart"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::MotionBAEnd());
            this->catalogMap["vision::vslamVerboseCpp::MotionBAEnd"] = std::string(umsg.begin(), umsg.end());

            //
            // Mapping
            //
            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::startMapping("%s"));
            this->catalogMap["vision::vslamVerboseCpp::startMapping"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::NumCulledPoints("%s"));
            this->catalogMap["vision::vslamVerboseCpp::NumCulledPoints"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::NumTrackedMapPointsAddedFrame("%s"));
            this->catalogMap["vision::vslamVerboseCpp::NumTrackedMapPointsAddedFrame"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::NewConnectionWithPoints("%s", "%s", "%s"));
            this->catalogMap["vision::vslamVerboseCpp::NewConnectionWithPoints"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::MaxDepthBeforeBA("%s", "%s"));
            this->catalogMap["vision::vslamVerboseCpp::MaxDepthBeforeBA"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::MaxDepthAfterBA("%s", "%s"));
            this->catalogMap["vision::vslamVerboseCpp::MaxDepthAfterBA"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::BAStart());
            this->catalogMap["vision::vslamVerboseCpp::BAStart"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::BAEnd());
            this->catalogMap["vision::vslamVerboseCpp::BAEnd"] = std::string(umsg.begin(), umsg.end());
            
            //
            // Loop closure
            //
            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::loopDetected("%s"));
            this->catalogMap["vision::vslamVerboseCpp::loopDetected"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::loopClosureStart());
            this->catalogMap["vision::vslamVerboseCpp::loopClosureStart"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::loopClosureEnd());
            this->catalogMap["vision::vslamVerboseCpp::loopClosureEnd"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::loopClosed("%s"));
            this->catalogMap["vision::vslamVerboseCpp::loopClosed"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::loopCandidates("%s"));
            this->catalogMap["vision::vslamVerboseCpp::loopCandidates"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::numKeyFramesSinceLastLoop("%s"));
            this->catalogMap["vision::vslamVerboseCpp::numKeyFramesSinceLastLoop"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::loopDetectionStart());
            this->catalogMap["vision::vslamVerboseCpp::loopDetectionStart"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::loopDetectionEnd());
            this->catalogMap["vision::vslamVerboseCpp::loopDetectionEnd"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::loopNotDetected("%s"));
            this->catalogMap["vision::vslamVerboseCpp::loopNotDetected"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::loopCandidateIdScore("%s", "%s", "%s"));
            this->catalogMap["vision::vslamVerboseCpp::loopCandidateIdScore"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::loopCandidateConnectedIdScore("%s", "%s", "%s"));
            this->catalogMap["vision::vslamVerboseCpp::loopCandidateConnectedIdScore"] = std::string(umsg.begin(), umsg.end());

            umsg = fl::i18n::MessageCatalog::get_message(vision::vslamVerboseCpp::loopNotClosed());
            this->catalogMap["vision::vslamVerboseCpp::loopNotClosed"] = std::string(umsg.begin(), umsg.end());
        }

        #pragma clang diagnostic push
        #pragma clang diagnostic ignored "-Wdeprecated-declarations"
        std::string VSLAMLogger::catalogFormat(const std::string& catalogKey, const std::vector<std::string>& args) {

            std::string buffer("");
            if(this->catalogMap.find(catalogKey) != this->catalogMap.end()) {

                buffer = this->catalogMap[catalogKey];
                int counterArgs = static_cast<int>(args.size());
                for(int i=0; i < static_cast<int>(args.size()); i++) {
                    auto sz = buffer.size();

                    // This is important to avoid any string truncation when inserting large vectors/matrices
                    buffer.resize(sz+args[i].size(),' ');

                    // It is important to use vector<char> to avoid ostream segfaults
                    std::vector<char> vec;
                    vec.assign(buffer.begin(), buffer.end());
                    // Adding delimiter is important to avoid string issues
                    vec.push_back('\0');
                    
                    // Number of "%s" do not matter. Three counts is just a safe number.
                    // Increase the number of "%s" if the catalog message takes more than
                    // three input arguments.
                    switch(counterArgs){
                        case 1: 
                            sprintf (&vec[0], buffer.c_str(), args[i].c_str());
                            break;
                        case 2:
                            sprintf (&vec[0], buffer.c_str(), args[i].c_str(), "%s");
                            break;
                        case 3:
                            sprintf (&vec[0], buffer.c_str(), args[i].c_str(), "%s", "%s");
                            break;
                        case 4:
                            sprintf (&vec[0], buffer.c_str(), args[i].c_str(), "%s", "%s", "%s");
                            break;
                        default:
                            break;
                    }
                    counterArgs--;
                    buffer = std::string(vec.begin(), vec.end());
                }
            } else if(static_cast<int>(args.size()) > 0) { // For simple logging of message
                buffer = args[0];
            }
            return buffer;
        }
        #pragma clang diagnostic pop

        void VSLAMLogger::logMessageV1(const std::string& catalogKey, const std::vector<std::string>& args) {

            if(this->verboseLevel == 1) { // On CMD
                std::string msg = catalogFormat(catalogKey, args);
                msgPrinter << msg.c_str() << std::endl;
            } else if(this->verboseLevel == 2) { // In File without timestamp
                std::string msg = catalogFormat(catalogKey, args);
                *this << msg.c_str() << std::endl;
            } else if(this->verboseLevel == 3) { // In File with timestamp
                std::string msg = catalogFormat(catalogKey, args);
                *this << std::time(nullptr) << " | " << msg.c_str() << std::endl;
            }
        }

        void VSLAMLogger::logMessageV2(const std::string& catalogKey, const std::vector<std::string>& args) {

            if(this->verboseLevel == 1) {
                // Level 2 messages are not printed when verbose level = 1
            } else if(this->verboseLevel == 2) { // In File without timestamp
                std::string msg = catalogFormat(catalogKey, args);
                *this << msg.c_str() << std::endl;
            } else if(this->verboseLevel == 3) { // In File with timestamp
                std::string msg = catalogFormat(catalogKey, args);
                *this << std::time(nullptr) << " | " << msg.c_str() << std::endl;
            }
        }

        void VSLAMLogger::logMessageV3(const std::string& catalogKey, const std::vector<std::string>& args) {

            if(this->verboseLevel == 1) {
                // Level 3 messages are not printed when verbose level = 1
            } else if(this->verboseLevel == 2) { // In File without timestamp
                // Level 3 messages are not printed when verbose level = 2
            } else if(this->verboseLevel == 3) { // In File with timestamp
                std::string msg = catalogFormat(catalogKey, args);
                *this << std::time(nullptr) << " | " << msg.c_str() << std::endl;
            }
        }
    } // namespace vslam
} // namespace vision