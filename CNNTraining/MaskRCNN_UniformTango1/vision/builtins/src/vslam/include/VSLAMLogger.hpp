////////////////////////////////////////////////////////////////////////////////
// Implementation for VSLAMLogger used for logging purposes in verbose mode.
// 
// Copyright 2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef VSLAMLOGGER_HPP
#define VSLAMLOGGER_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <cstdarg>
#include <unordered_map>
#include <vector>
#include <memory>

#include "i18n/MessageCatalog.hpp"
#include "resources/vision/vslamVerboseCpp.hpp"
#include "vslamcore/VSLAMLoggerBase.hpp"
#include "services/io/unicode_stream.hpp"

namespace vision {
    namespace vslam {

        class VSLAMLogger : public VSLAMLoggerBase {

        public:
            /**
            * @brief Constructor for CMD mode
            */
            VSLAMLogger(const int _verboseLevel);

            /**
            * @brief Constructor for File mode
            */
            VSLAMLogger(const int _verboseLevel, const std::string& _filename);

            /**
            * @brief The function decides how to log a level 1 messages 
            *        when the verbose level is set to 1, 2 or 3.
            *
            * @param[in] catalogKey catalog message key
            * @param[in] args Message catalog arguments
            */
            void logMessageV1(const std::string& catalogKey, const std::vector<std::string>& args = std::vector<std::string>()) override;

            /**
            * @brief The function decides how to log a level 2 messages
            *         when the verbose level is set to 1, 2 or 3.
            *
            * @param[in] catalogKey catalog message key
            * @param[in] args Message catalog arguments
            */
            void logMessageV2(const std::string& catalogKey, const std::vector<std::string>& args = std::vector<std::string>()) override;

            /**
            * @brief The function decides how to log a level 3 messages
            *         when the verbose level is set to 1, 2 or 3.
            *
            * @param[in] catalogKey catalog message key
            * @param[in] args Message catalog arguments
            */
            void logMessageV3(const std::string& catalogKey, const std::vector<std::string>& args = std::vector<std::string>()) override;

            /**
            * @brief Destructor
            */
            ~VSLAMLogger() { }

        protected:

            /**
            * @brief The function converts catalog key and arguments into
            *        one message.
            *
            * @param[in] catalogKey catalog message key
            * @param[in] args Message catalog arguments
            */
            std::string catalogFormat(const std::string& catalogKey, const std::vector<std::string>& args);

            /**
            * @brief The function converts catalog key and arguments into
            *        one message.
            */
            void initializeCatalogMap();

            /**
            * Map to store catalog key to catalog message
            */
            std::unordered_map<std::string, std::string> catalogMap;

            /**
            * To print on command window;
            */
            services::io::uout msgPrinter;
        };
    }// namespace vslam
}// namespace vision

#endif // VSLAMLOGGER_HPP
