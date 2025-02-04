////////////////////////////////////////////////////////////////////////////////
// Base class for VSLAMLogger used for logging purposes in verbose mode. 
// 
// Copyright 2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef VSLAMLOGGERBASE_HPP
#define VSLAMLOGGERBASE_HPP

#include <cstdarg>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

namespace vision {
    namespace vslam {
        
        class VSLAMLoggerBase : public std::ofstream {

        public:
            /**
            * @brief Constructor for CMD mode
            */
            VSLAMLoggerBase(const int _verboseLevel) : verboseLevel(_verboseLevel), filename("") { }

            /**
            * @brief Constructor for File mode
            */
            VSLAMLoggerBase(const int _verboseLevel, const std::string& _filename) 
                : std::ofstream(_filename.c_str(), std::ofstream::out), verboseLevel(_verboseLevel), filename(_filename) { }

            /**
            * @brief The function decides how to log level 1 messages 
            *        when the verbose level is set to 1, 2 or 3.
            *
            * @param[in] catalogKey catalog message key
            * @param[in] args Message catalog arguments
            */
            virtual void logMessageV1(const std::string& catalogKey, const std::vector<std::string>& args = std::vector<std::string>()) {
                /* Do nothing */
            }

            /**
            * @brief The function decides how to log level 2 messages
            *         when the verbose level is set to 1, 2 or 3.
            *
            * @param[in] catalogKey catalog message key
            * @param[in] args Message catalog arguments
            */
            virtual void logMessageV2(const std::string& catalogKey, const std::vector<std::string>& args = std::vector<std::string>()) {
                /* Do nothing */
            }

            /**
            * @brief The function decides how to log level 3 messages
            *         when the verbose level is set to 1, 2 or 3.
            *
            * @param[in] catalogKey catalog message key
            * @param[in] args Message catalog arguments
            */
            virtual void logMessageV3(const std::string& catalogKey, const std::vector<std::string>& args = std::vector<std::string>()) {
                /* Do nothing */
            }

            /**
            * @brief Return the log filename
            *
            * @return Log filename
            */
            std::string getLogFileName() const {
                return this->filename;
            }

            /**
            * @brief Return the verbose level
            *
            * @return Verbose level
            */
            int getVerboseLevel() const {
                return this->verboseLevel;
            }
            
            /**
            * @brief Destructor
            */
            virtual ~VSLAMLoggerBase() {
                if(this->verboseLevel > 1) {
                    this->close();
                }
            }
        protected:
            /**
            * Verbose level
            */
            int verboseLevel;

            /**
            * Log file name
            */
            std::string filename;
        };
    }// namespace vslam
}// namespace vision

#endif // VSLAMLOGGERBASE_HPP
