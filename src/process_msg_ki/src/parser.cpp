//
// Created by rukangxu on 14.12.21.
//

#include <iostream>
#include <fstream>
#include <sstream>

#include <string>
#include <stdexcept>
#include <list>

#include <sys/types.h>
#include <sys/stat.h>
//#include <unistd.h>
//#include <errno.h>
#include <cerrno>
#include <xercesc/framework/MemBufInputSource.hpp>

#include "parser.hpp"

using namespace xercesc;
using namespace std;


/**
 * \brief read the file (.xml in our case) to string
 * \param path to the file
 * \return string read from the file
 */
string readFileIntoString(const string& path) {
    ifstream input_file(path);
    if (!input_file.is_open()) {
        cerr << "Could not open the file - '"
             << path << "'" << endl;
        exit(EXIT_FAILURE);
    }
    return string((std::istreambuf_iterator<char>(input_file)), std::istreambuf_iterator<char>());
}

GetConfig::GetConfig()
{
    // Initialize xerces
    try
    {
        XMLPlatformUtils::Initialize();  // Initialize Xerces infrastructure
    }
    catch( XMLException& e )
    {
        char* message = XMLString::transcode( e.getMessage() );
        cerr << "XML toolkit initialization error: " << message << endl;
        XMLString::release( &message );
        // throw exception here to return ERROR_XERCES_INIT
    }

    // Tags and attributes used in XML file.
    // Can't call transcode till after Xerces Initialize()

    TAG_Rob = XMLString::transcode("Rob");
    TAG_RIst = XMLString::transcode("RIst");
    TAG_IPOC = XMLString::transcode("IPOC");

    m_ConfigFileParser = new XercesDOMParser;
    m_StringParser = new XercesDOMParser;

}

GetConfig::~GetConfig()
{
    // Free memory

    delete m_ConfigFileParser;
    delete m_StringParser;
    if(strlen(rob_ipoc)>0)    XMLString::release( &rob_ipoc );
    // TODO may need to add condition for memory release of the following two objs
    doc->release();
    output->release();
    XMLString::release(&c_ptr);

    try
    {
        XMLString::release( &TAG_Rob );
        XMLString::release( &TAG_RIst );
        XMLString::release( &TAG_IPOC );
    }
    catch( ... )
    {
        cerr << "Unknown exception encountered in TagNamesdtor" << endl;
    }

    // Terminate Xerces

    try
    {
        XMLPlatformUtils::Terminate();  // Terminate after release of memory
    }
    catch( XMLException& e )
    {
        char* message = XMLString::transcode( e.getMessage() );

        cerr << "XML ttolkit teardown error: " << message << endl;
        XMLString::release( &message );
    }
}

void GetConfig::readConfigFile(string& configFile)
noexcept(false)
{
    // Test to see if the file is ok.

    struct stat fileStatus;

    errno = 0;
    if(stat(configFile.c_str(), &fileStatus) == -1) // ==0 ok; ==-1 error
    {
        if( errno == ENOENT )      // errno declared by include file errno.h
            throw ( std::runtime_error("Path file_name does not exist, or path is an empty string.") );
        else if( errno == ENOTDIR )
            throw ( std::runtime_error("A component of the path is not a directory."));
        else if( errno == ELOOP )
            throw ( std::runtime_error("Too many symbolic links encountered while traversing the path."));
        else if( errno == EACCES )
            throw ( std::runtime_error("Permission denied."));
        else if( errno == ENAMETOOLONG )
            throw ( std::runtime_error("File can not be read\n"));
    }

    // Configure DOM parser.
    // TODO what are the configurations on earth??
    m_ConfigFileParser->setValidationScheme( XercesDOMParser::Val_Never );
    m_ConfigFileParser->setDoNamespaces( false );
    m_ConfigFileParser->setDoSchema( false );
    m_ConfigFileParser->setLoadExternalDTD( false );

    try
    {
        m_ConfigFileParser->parse( configFile.c_str() );

        // no need to free this pointer - owned by the parent parser object
        DOMDocument* xmlDoc = m_ConfigFileParser->getDocument();

        // Get the top-level element: Name is "root". No attributes for "root"

        DOMElement* elementRoot = xmlDoc->getDocumentElement();
        if( !elementRoot ) throw(std::runtime_error( "empty XML document" ));

        // Parse XML file for tags of interest: "IPOC"
        // Look one level nested within "Rob". (child of root)

        DOMNodeList*      children = elementRoot->getChildNodes();
        const  XMLSize_t  nodeCount = children->getLength();

        // For all nodes, children of "root" in the XML tree.

        for( XMLSize_t xx = 0; xx < nodeCount; ++xx )
        {
            DOMNode* currentNode = children->item(xx);
            if( currentNode->getNodeType() &&  // true is not NULL
                currentNode->getNodeType() == DOMNode::ELEMENT_NODE ) // is element
            {
                // Found node which is an Element. Re-cast node as element
                DOMElement* currentElement
                        = dynamic_cast< DOMElement* >( currentNode );
                if( XMLString::equals(currentElement->getTagName(), TAG_IPOC))
                {
                    // Already tested node as type element and of name "IPOC".
                    // Read attributes of element "ApplicationSettings".
//                    const XMLCh* xmlch_OptionA
//                            = currentElement->getAttribute(ATTR_OptionA);
//                    m_OptionA = XMLString::transcode(xmlch_OptionA);
//
//                    const XMLCh* xmlch_OptionB
//                            = currentElement->getAttribute(ATTR_OptionB);
//                    m_OptionB = XMLString::transcode(xmlch_OptionB);

                    const XMLCh* xmlch_rob_ipoc
                            = currentElement->getTextContent();
                    rob_ipoc = XMLString::transcode(xmlch_rob_ipoc);

                    break;  // Data found. No need to look at other elements in tree.
                }
            }
        }
    }
    catch( XMLException& e )
    {
        char* message = XMLString::transcode( e.getMessage() );
        ostringstream errBuf;
        errBuf << "Error parsing file: " << message << flush;
        XMLString::release( &message );
    }
}

void GetConfig::readConfigOnFly(string& myxml)
noexcept(false)
{
    bool interestParts[2] = {false, false};
    MemBufInputSource myxml_buf(reinterpret_cast<const XMLByte *const>(myxml.c_str()),
                                         myxml.size(), "myxml (in memory)" );

    // Configure DOM parser.
    m_StringParser->setValidationScheme( XercesDOMParser::Val_Never );
    m_StringParser->setDoNamespaces( false );
    m_StringParser->setDoSchema( false );
    m_StringParser->setLoadExternalDTD( false );

    try
    {
        m_StringParser->parse( myxml_buf );

        // no need to free this pointer - owned by the parent parser object
        DOMDocument* xmlDoc = m_StringParser->getDocument();

        // Get the top-level element: Name is "root". No attributes for "root"
        DOMElement* elementRoot = xmlDoc->getDocumentElement();
        if( !elementRoot ) throw(std::runtime_error( "empty XML string found" ));

        // Look one level nested within "Rob". (child of root)
        DOMNodeList*      children = elementRoot->getChildNodes();
        const  XMLSize_t  nodeCount = children->getLength();

        // For all nodes, children of "root" in the XML tree.

        for( XMLSize_t xx = 0; xx < nodeCount; ++xx )
        {
            DOMNode* currentNode = children->item(xx);
            auto* parentElement = dynamic_cast< DOMElement* >( currentNode->getParentNode() );
            if( currentNode->getNodeType() &&  // true when not being NULL
                currentNode->getNodeType() == DOMNode::ELEMENT_NODE &&  // true when being element
                XMLString::equals(parentElement->getTagName(), TAG_Rob) ) // true when parent being Rob
            {
                // Found node which is an Element. Re-cast node as element
                auto* currentElement
                          = dynamic_cast< DOMElement* >( currentNode );
                // Parse XML file for tags of interest: "IPOC" and "TAG_RIst" (now not activated)
                if( XMLString::equals(currentElement->getTagName(), TAG_IPOC))
                {
                    // Already tested node as type element and of name "IPOC".
                    interestParts[0] = true;
                    const XMLCh* xmlch_rob_ipoc = currentElement->getTextContent();
                    rob_ipoc = XMLString::transcode(xmlch_rob_ipoc);
                }
                // TODO Parse XML file for tags of interest: "TAG_RIst" (can be activated if needed!)
                else if ( XMLString::equals(currentElement->getTagName(), TAG_RIst) )
                {
                    interestParts[1] = true;
                    std::vector<std::string> r_ist;
                    auto *pb = begin(attr_r_ist);
                    auto *pe = end(attr_r_ist);
                    // TODO to figure out why there are 7 elements of attr_r_ist added
                    for ( pb; pb!=pe-1; ++pb) {
                        char temp[] = {*pb, '\0'};
                        XMLCh* attr_key =  XMLString::transcode(temp);
                        char* attr_value = XMLString::transcode(currentElement->getAttribute(attr_key));
                        r_ist.emplace_back( string(attr_value) );
                    }
                    string tag_r_ist(XMLString::transcode(TAG_RIst));
                    extract_data[tag_r_ist] = r_ist;
                }
            }
        }
    }
    catch( XMLException& e )
    {
        char* message = XMLString::transcode( e.getMessage() );
        ostringstream errBuf;
        errBuf << "Error parsing file: " << message << flush;
        XMLString::release( &message );
    }
}

int GetConfig::createDOMString()
{
    int rtn = 0;
    DOMImplementation* impl = DOMImplementationRegistry::getDOMImplementation(
            XMLString::transcode("Core"));
    if (impl == nullptr)
    {
        std::cout << "Implementation Error !" << std::endl;
        rtn = 4;
        return rtn;
    }

    try
    {
        // root element namespace URI.
        // root element name
        // document type object (DTD).
        /* the sample xml data sent back to robot
         * <?xml version="1.0" encoding="UTF-8"?>
         * <Sen Type="ImFree">
         *     <RKorrY>0.0</RKorrY>
         *     <RKorrX>0.0</RKorrX>
         *     <RKorrZ>0.0</RKorrZ>
         *     <IPOC>rob_ipoc</IPOC>
         * </Sen>
         */

        doc = impl->createDocument( nullptr, XMLString::transcode("Sen"), nullptr );
        DOMElement* rootElem = doc->getDocumentElement();
        rootElem->setAttribute(XMLString::transcode("Type"), XMLString::transcode("ImFree"));

        DOMElement*  RKorrYElem = doc->createElement(XMLString::transcode("RKorrY"));
        DOMText*  RKorrYText = doc->createTextNode(XMLString::transcode("0.0"));
        RKorrYElem->appendChild(RKorrYText);
        rootElem->appendChild(RKorrYElem);

        DOMElement*  RKorrXElem = doc->createElement(XMLString::transcode("RKorrX"));
        DOMText* RKorrXText = doc->createTextNode(XMLString::transcode("0.0"));
        RKorrXElem->appendChild(RKorrXText);
        rootElem->appendChild(RKorrXElem);

        DOMElement*  RKorrZElem = doc->createElement(XMLString::transcode("RKorrZ"));
        DOMText* RKorrZText = doc->createTextNode(XMLString::transcode("0.0"));
        RKorrZElem->appendChild(RKorrZText);
        rootElem->appendChild(RKorrZElem);

        DOMElement*  IPOCElem = doc->createElement(XMLString::transcode("IPOC"));
        DOMText* IPOCText = doc->createTextNode(XMLString::transcode(rob_ipoc));
        IPOCElem->appendChild(IPOCText);
        rootElem->appendChild(IPOCElem);
    }
    catch (...)
    {
        std::cout << "An error occurred creating the document" << std::endl;
        rtn = 3;
    }

    return rtn;
}

void GetConfig::DoOutput2Stream(DOMDocument* pmyDOMDocument, bool debug)
{
    // NOTE: XERCES_VERSION_MAJOR == 3

    DOMImplementation*    pImplement    = nullptr;
    DOMLSSerializer*       pSerializer    = nullptr;
    /*
    Return the first registered implementation that has
    the desired features. In this case, we are after
    a DOM implementation that has the LS feature... or Load/Save.
    */
    pImplement = DOMImplementationRegistry::getDOMImplementation(XMLString::transcode("LS"));

    /*
    From the DOMImplementation, create a DOMLSSerializer instead of DOMWriter
    since the latter is removed upon upgrade.
    DOMLSSerializer are used to serialize a DOM tree [back] into an XML document.
    */
    pSerializer = ((DOMImplementationLS*)pImplement)->createLSSerializer();

    /*
    Choose a location for the serialized output. The 3 options are:
    1) StdOutFormatTarget     (std output stream -  good for debugging)
    2) MemBufFormatTarget     (to Memory)
    3) LocalFileFormatTarget  (save to file)
    (Note: You'll need a different header file for each one)
    */
    if (!debug){
        auto *pMBFTarget = new MemBufFormatTarget();
        output = ((DOMImplementationLS*)pImplement)->createLSOutput();
        output->setByteStream(pMBFTarget);
        pSerializer->write(pmyDOMDocument, output);
        data = pMBFTarget->getRawBuffer();
        data_len = pMBFTarget->getLen();
        // convert from the encoding of the service to the internal XMLCh * encoding
        TranscodeFromStr transcodeFrom(data, data_len, "UTF-8");
        // TODO to check the difference between transcodeFrom.str() and transcodeFrom.adopt() since the latter causes that xmlch_ptr points to null c_string
        const XMLCh* xmlch_ptr = transcodeFrom.str();
        c_ptr = XMLString::transcode(xmlch_ptr);
        // release memory
        pSerializer->release();
        delete pMBFTarget;
    } else {
        //The end-of-line sequence of characters to be used in the XML being written out.
        //pSerializer->setNewLine(XMLString::transcode("\n"));
        DOMConfiguration* pConfiguration = pSerializer->getDomConfig();
        /*
        This line is optional. It just sets a feature
        of the Serializer to make the output
        more human-readable by inserting line-feeds and tabs,
        without actually inserting any new nodes
        into the DOM tree. (There are many different features to set.)
        Comment it out and see the difference.
        */
        if (pConfiguration->canSetParameter(XMLUni::fgDOMWRTFormatPrettyPrint, true))
            pConfiguration->setParameter(XMLUni::fgDOMWRTFormatPrettyPrint, true);
        auto *pSOFTarget = new StdOutFormatTarget();
        // auto *pLFFTarget = new LocalFileFormatTarget(f_name);  // for using LocalFileFormatTarget
        output = ((DOMImplementationLS*)pImplement)->createLSOutput();
        output->setByteStream(pSOFTarget);
        pSerializer->write(pmyDOMDocument, output);
        // release memory
        pSerializer->release();
        delete pSOFTarget;
    }
}

//#ifdef MAIN_TEST
/* This main is provided for unit test of the class. */

//int main(int argc, char * argv[])
//{
//    string configFile="/home/rukangxu/rkx/ias_hiwi/build_interface/sample_kuka.xml"; // stat file. Get ambigious segfault otherwise.
//    string configString = readFileIntoString(configFile);
//
//    GetConfig appConfig;
//
//    // appConfig.readConfigFile(configFile);
//    appConfig.readConfigOnFly(configString);
//
//    cout << "Rob IPOC = "  << appConfig.getRobIPOC()  << endl;
//    return 0;
//}
//#endif