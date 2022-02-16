//
// Created by rukangxu on 14.12.21.
//

#ifndef BUILD_INTERFACE_PARSER_H
#define BUILD_INTERFACE_PARSER_H


/**
 *  @file
 *  Class "GetConfig" provides the functions to read the XML data.
 *  @version 1.0
 */
// NOTE: XERCES_VERSION_MAJOR == 3
#include <xercesc/dom/DOM.hpp>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMDocumentType.hpp>
#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMImplementationLS.hpp>
#include <xercesc/dom/DOMNodeIterator.hpp>
#include <xercesc/dom/DOMNodeList.hpp>
#include <xercesc/dom/DOMText.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/util/XMLUni.hpp>
#include <xercesc/util/XMLString.hpp>
// Mandatory for using any feature of Xerces.
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/TransService.hpp>
// Required for outputing a Xerces DOMDocument
// to a standard output stream (Also see: XMLFormatTarget)
#include <xercesc/framework/StdOutFormatTarget.hpp>
#include <xercesc/framework/MemBufFormatTarget.hpp>

#include <string>
#include <stdexcept>
#include <map>
#include <vector>



// Error codes
enum {
    ERROR_ARGS = 1,
    ERROR_XERCES_INIT,
    ERROR_PARSE,
    ERROR_EMPTY_DOCUMENT
};

class GetConfig
{
public:
    /**
      * \brief Constructor initializes xerces-C libraries.
      * \brief The XML tags and attributes which we seek are defined.
      * \brief The xerces-C DOM parser infrastructure is initialized.
      */
    GetConfig();
    /**
      * \brief Class destructor frees memory used to hold the XML tag and
      * \brief attribute definitions. It als terminates use of the xerces-C
      * \brief framework.
      */
    ~GetConfig();
    /**
      * \brief read data in xml format from file (for testing)
      * \brief - Tests the access and availability of the XML configuration file.
      * \brief - Configures the xerces-c DOM parser.
      * \brief - Reads and extracts the pertinent information from the XML config file.
      * \param in configFile The text string name of the HLA configuration file.
      */
    void readConfigFile(std::string&) noexcept(false);
    /**
      * \brief read data in string instead of from file
      * \param string representing data
      */
    void readConfigOnFly(std::string&) noexcept(false);
    /**
     * \brief create a new DOM document based on the processed results to be sent back
     * \return error code indicating the success or failure of function execution
     */
    int createDOMString();
    /**
     * \brief get the ipoc info
     * \return ipoc in c_string
     */
    char *getRobIPOC() { return rob_ipoc; };
    /**
     * \brief output the resulting DOM document on a console or to a file
     * \param pointer to the resulting DOM document
     * \param debugging mode is activated if true
     */
    void DoOutput2Stream(xercesc_3_2::DOMDocument* pmyDOMDocument, bool debug);
    xercesc_3_2::DOMDocument* doc = nullptr;
    xercesc_3_2::DOMLSOutput* output = nullptr;
    const XMLByte* data = nullptr;
    XMLSize_t data_len = 0;
    char* c_ptr = nullptr;
    std::map<std::string, std::vector<std::string> > extract_data;

private:
    xercesc::XercesDOMParser *m_ConfigFileParser;
    xercesc::XercesDOMParser *m_StringParser;
    char* rob_ipoc = nullptr;
    char attr_r_ist[7] = "XYZABC";

    // Internal class use only. Hold Xerces data in UTF-16 XMLCh type.
    XMLCh* TAG_Rob;
    XMLCh* TAG_RIst;
    XMLCh* TAG_IPOC;

};

#endif //BUILD_INTERFACE_PARSER_H
