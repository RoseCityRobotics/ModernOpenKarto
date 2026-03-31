/*
 * Karto (tm) Robot Navigation Software - Software Development Kit
 * Release 2.1
 *
 * Copyright (C) 2006-2011, SRI International (R)
 *
 * The material contained in this release is copyrighted. It may not be copied, 
 * reproduced, translated, reverse engineered, modified or reduced to any electronic
 * medium or machine-readable form without the prior written consent of
 * SRI International (R).
 *
 * Portions of files in this release may be unpublished work 
 * containing SRI International (R) CONFIDENTIAL AND PROPRIETARY INFORMATION. 
 * Disclosure, use, reverse engineering, modification, or reproduction without
 * written authorization of SRI International (R) is likewise prohibited.
 *
 * Karto (tm) is a Trademark of SRI International (R).
 *
 * Author(s): Michael A. Eriksen (eriksen@ai.sri.com)
 */

#include <iostream>

#ifdef USE_POCO
#include <Poco/Logger.h>
#include <Poco/LogStream.h>
#include <Poco/PatternFormatter.h>
#include <Poco/FormattingChannel.h>
#include <Poco/ConsoleChannel.h>
#include <Poco/FileChannel.h>
#include <Poco/TemporaryFile.h>
#include <Poco/Process.h>
#include <Poco/Path.h>
#endif

#include <Logger.h>

namespace karto
{

  static kt_bool s_FileLoggingEnabled = false;
  static LogLevel s_LogLevel = LOG_NONE;

  BasicEvent<LogMessageArguments> LogMessage;

  void InitializeLogger(const std::string& rApplicationName, const std::string& rLogLevel)
  {
    LogLevel logLevel = LOG_NONE;

    if (rLogLevel.find("NONE") != std::string::npos) 
    {
      logLevel = LOG_NONE;
    }
    else if (rLogLevel.find("FATAL") != std::string::npos) 
    {
      logLevel = LOG_FATAL;
    }
    else if (rLogLevel.find("ERROR") != std::string::npos) 
    {
      logLevel = LOG_ERROR;
    }
    else if (rLogLevel.find("WARNING") != std::string::npos) 
    {
      logLevel = LOG_WARNING;
    }
    else if (rLogLevel.find("INFORMATION") != std::string::npos) 
    {
      logLevel = LOG_INFORMATION;
    }
    else if (rLogLevel.find("DEBUG") != std::string::npos) 
    {
      logLevel = LOG_DEBUG;
    }
    else 
    {
      std::cerr << "Warning: invalid KARTO_LOG_LEVEL [" << rLogLevel << "] using NONE as default!"<<std::endl;
    }

    std::string logName;
    if (logLevel != LOG_NONE)
    {
     if (s_FileLoggingEnabled == true)
      {
      // create log filename
      logName = rApplicationName + ".log";

#ifdef USE_POCO
      {
        // test if we have write permission!!
        try
        {
          std::ostringstream name;
          unsigned long n = 1;

          name << "tmp" << Poco::Process::id();
          for (int i = 0; i < 6; ++i)
          {
            name << char('a' + (n % 26));
            n /= 26;
          }

          Poco::File dummyFile(Poco::Path::current() + name.str());

          // check that we actually can write!
          dummyFile.createFile();
          dummyFile.remove();
          s_FileLoggingEnabled = true;
        }
        catch (Poco::FileAccessDeniedException error)
        {
          s_FileLoggingEnabled = false;
        }
      }
     }

      // create loggers
      Poco::FormattingChannel* pFCConsole = new Poco::FormattingChannel(new Poco::PatternFormatter("%t"));
      pFCConsole->setChannel(new Poco::ConsoleChannel);
      pFCConsole->open();

      Poco::FormattingChannel* pFCFile = NULL;
      if (s_FileLoggingEnabled == true)
      {
        Poco::PatternFormatter* pFormatter = new Poco::PatternFormatter("%Y-%m-%d %H:%M:%S.%c %N[%P]:%q:%t");
        pFormatter->setProperty("times", "local");
        pFCFile = new Poco::FormattingChannel(pFormatter);
        pFCFile->setChannel(new Poco::FileChannel(logName.c_str()));
        pFCFile->open();
      }

#ifdef _DEBUG
      Poco::Logger::create("ConsoleLogger", pFCConsole, Poco::Message::PRIO_DEBUG);
      if (pFCFile)
      {
        Poco::Logger::create("FileLogger", pFCFile, Poco::Message::PRIO_DEBUG);
      }
#else
      Poco::Logger::create("ConsoleLogger", pFCConsole, Poco::Message::PRIO_INFORMATION);
      if (pFCFile)
      {
        Poco::Logger::create("FileLogger", pFCFile, Poco::Message::PRIO_INFORMATION);
      }
#endif // _DEBUG
#else
     }
#endif // USE_POCO
    }

    SetLogLevel(logLevel);

    if (logLevel != LOG_NONE)
    {
      if (s_FileLoggingEnabled)
      {
        Log(LOG_DEBUG, "Karto created log file: " + logName);
      }
      else
      {
        if (logName != "") 
          Log(LOG_INFORMATION, "Karto unable to create log file [" + logName + "]. No write permission to log directory.");
      }
    }
  }

  void SetLogLevel(LogLevel level)
  {
    s_LogLevel = level;

    if (s_LogLevel != LOG_NONE)
    {
#ifdef USE_POCO
      Poco::Logger::get("ConsoleLogger").setLevel(level);
      Poco::Logger::get("FileLogger").setLevel(level);
#endif // USE_POCO
    }
  }

  LogLevel GetLogLevel()
  {
    return s_LogLevel;
  }

  void Log(LogLevel level, const std::string& rMessage)
  {
    if (s_LogLevel != LOG_NONE)
    {
#ifdef USE_POCO
      if (level == LOG_FATAL)
      {
        Poco::Logger::get("ConsoleLogger").fatal(rMessage.c_str());
        Poco::Logger::get("FileLogger").fatal(rMessage.c_str());
      }
      else if (level == LOG_ERROR)
      {
        Poco::Logger::get("ConsoleLogger").error(rMessage.c_str());
        Poco::Logger::get("FileLogger").error(rMessage.c_str());
      }
      else if (level == LOG_WARNING)
      {
        Poco::Logger::get("ConsoleLogger").warning(rMessage.c_str());
        Poco::Logger::get("FileLogger").warning(rMessage.c_str());
      }
      else if (level == LOG_INFORMATION)
      {
        Poco::Logger::get("ConsoleLogger").information(rMessage.c_str());
        Poco::Logger::get("FileLogger").information(rMessage.c_str());
      }
      else if (level == LOG_DEBUG)
      {
        Poco::Logger::get("ConsoleLogger").debug(rMessage.c_str());
        Poco::Logger::get("FileLogger").debug(rMessage.c_str());
      }
      else
      {
        Poco::Logger::get("ConsoleLogger").information(rMessage.c_str());
        Poco::Logger::get("FileLogger").information(rMessage.c_str());
      }
#else
      std::cout << "Warning OpenKarto is compiled without POCO, so no logging enabled! Compile with POCO and define USE_POCO to enable logging." << std::endl;
      std::cout << rMessage << std::endl;
#endif // USE_POCO

      LogMessageArguments eventArguments(level, rMessage);
      LogMessage.Notify(NULL, eventArguments);
    }
  }

  void TerminateLogger()
  {
    if (s_LogLevel != LOG_NONE)
    {
#ifdef USE_POCO
      Poco::Logger::get("ConsoleLogger").close();

      Poco::Channel* pChannel = Poco::Logger::get("FileLogger").getChannel();
      if (pChannel != NULL)
      {
        pChannel->close();
      }

      Poco::Logger::get("ConsoleLogger").shutdown();
      Poco::Logger::get("FileLogger").shutdown();
      Poco::Logger::shutdown();
#endif // USE_POCO
    }
  }

}
