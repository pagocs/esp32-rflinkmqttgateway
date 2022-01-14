//------------------------------------------------------------------------------
// RFLINK MQTT gateway ESP32 firmware
// Author pagocs
// Licence: MIT
//------------------------------------------------------------------------------

#include "yoursettings.h"

//------------------------------------------------------------------------------

#include <ESP.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <string.h>
#include <soc/rtc.h>
#include <ArduinoJson.h>

//------------------------------------------------------------------------------
// Common modules

#include <utils.h>
#include <ota.h>
#include <ledblink.H>
#include <wificonnect.H>
#include <mqtt.h>
#include <wol.h>

#ifndef DEBUG_PRINTF
#define DEBUG_PRINTF(f_, ...) Serial.printf((f_), ##__VA_ARGS__)
//#define DEBUG_PRINTF(f_, ...)
#endif //DEBUG_PRINTF
#ifndef DEBUG_MODE
#define DEBUG_MODE() false
#endif // DEBUG_PRINTF

//------------------------------------------------------------------------------

#define BAUD     115200                     // Baudrate for serial communication.

//------------------------------------------------------------------------------
#include <HardwareSerial.h>
#define PIN_SERIAL_RX  16
#define PIN_SERIAL_TX  17
// https://stackoverflow.com/questions/49212371/serial-communication-between-two-esp32
//HardwareSerial rflink(2); // use uart2
HardwareSerial rflink(1); // use uart1

//------------------------------------------------------------------------------
// Syslog is optional. If you do not define the server it is not compile to
// the project
#ifdef SYSLOG_SERVER
#include <Syslog.h>
// This device info
#define DEVICE_HOSTNAME "ESP32 RFLINK MQTT"
#define APP_NAME "RFLINK MQTT Gateway"
//Source:
// https://github.com/arcao/Syslog
// https://github.com/arcao/Syslog/blob/master/examples/SimpleLoggingWiFi/SimpleLoggingWiFi.ino
// A UDP instance to let us send and receive packets over UDP
WiFiUDP udpClient;
// Create a new syslog instance with LOG_KERN facility
Syslog syslog(udpClient, SYSLOG_SERVER, SYSLOG_PORT, DEVICE_HOSTNAME, APP_NAME, LOG_KERN);
#define SYSLOG( s__ , t__ , ... ) syslog.logf( (s__) , (t__) , ##__VA_ARGS__ )
#else
#define SYSLOG( s__ , t__ , ... )
#endif

//----------------------------------------------------------------------

char esp32devicename[32];
#define MAX_DEVICENAME_LENGTH sizeof(esp32devicename)

//----------------------------------------------------------------------
// RFLINK MQTT related stuff

#define TEST_TOPIC    "test"
#define RFLINK_RECEIVE_TOPIC    "rflink/in"
#define RFLINK_CONTROL_TOPIC    "rflink/out"


#define RFLINK_COMMANDSENDMILLIS 1000
//----------------------------------------------------------------------
// Global variables

unsigned long   rflastreceived = 0;
unsigned long   rflastsent = 0;
unsigned long   rfheratbit = 0;
unsigned long   rflastreset = 0;
unsigned long   rfbroadcast = 0;
unsigned long   rfbroadcastrnd = 0; // Randomised timeshift
bool            rfbroadcastinitdelay = true;
unsigned int    rfpartialpackage = 0;
String          serialimput;
bool            rflinkpendingcommand = false;
unsigned long   rflinkmasterheartbit = ULONG_MAX;

#ifdef MULTIGATEWAY
bool            rflinkmastermode = false;
bool            rflinkknownmaster = false;
#endif // MULTIGATEWAY

//------------------------------------------------------------------------------
// Serial buffer and queu relatad stuff

#define RFLINK_WELCOME "20;00;Nodo RadioFrequencyLink"
#define MAX_RFPACKET_FIELDS 6

int serialbuffpos = 0;
char serialbuff[2048];
bool debugpacket = false;
bool rflinkbuffoverflow = false;

SemaphoreHandle_t mutex_rflinkqueue = NULL;

class RFLINKqueueitem {
public:
    time_t  timing;   // Send timing
    unsigned short retry;
    String  topic;
    String  payload;
    RFLINKqueueitem( const char * msgtopic , const char * msg , time_t delay = 0 )
    {
        timing = (delay == 0) ? 0 : time( NULL ) + delay;
        rprintf( "QUEUEitem: time:%d delay: %d timing: %d\n" , time( NULL ) , delay , timing );
        topic = String(msgtopic);
        payload = String( msg );
        retry = 0;
    };
};

std::vector<RFLINKqueueitem*> RFLINKqueue;

//------------------------------------------------------------------------------
// RFLINK serial communication wrapper functions

void rflinkinit( void )
{
    // Init work variables
    serialbuffpos = 0;
    debugpacket = false;
    rflastreceived = 0;
    rflastsent = 0;
    rfheratbit = 0;
    rfpartialpackage = 0;
    rflastreset = 0;
    rflinkbuffoverflow = false;
    rflinkpendingcommand = false;
    // init serial connection
    rflink.begin(57600, SERIAL_8N1, PIN_SERIAL_RX , PIN_SERIAL_TX );
    // TODO: 2018.08.03. ON ESP32 this method is not available
    //rflink.setRxBufferSize(1024);
    rflink.flush();
}

void rflinkreset( void )
{
    rflink.end();
    rflinkinit();
    rflastreset = millis();
}

unsigned long rflinklastreset()
{
    return (millis() - rflastreset );
}

unsigned long rflinklastsent()
{
    return ( millis() - rflastsent );
}


void rflinkend( )
{
    rflink.end();
}

size_t rflinkdataavailable( void )
{
    return rflink.available();
}

size_t rflinksend( const char *packet )
{
    // int i;
    // if( (i=rflinkdataavailable()) )
    // {
    //     rprintf( "!!! Serial data available while send (bytes: %d)....\n" , i);
    // }

    // Avoid the packet overlap
    if( rflinkpendingcommand == true && rflinklastsent() < RFLINK_COMMANDSENDMILLIS )
    {
        rprintf("!!! NOTICE: Wait for last command will send...\n" );

        do
        {
            delay(24);
        } while( rflinklastsent() < RFLINK_COMMANDSENDMILLIS && rflinkpendingcommand == true );
    }

    ledblink();
    rflink.flush();
    size_t sent = rflink.println( packet );
    rflink.flush();
    rflastsent = millis();
    rflinkpendingcommand = true;
    // rprintf( "%s: >>> RFLINK commad sent: %s (wrote: %d)\n" , timeClient.getFormattedTime().c_str() , packet , sent );
    return sent;
}

size_t rflinksend( String msg )
{
    //return rflink.write(  (const uint8_t*)msg.c_str(), msg.length() );
    return  rflinksend( msg.c_str() );
}

bool rflinkisdebugpacket( const char * packet )
{
    if( strstr( packet , ";DEBUG;" ) != NULL ||
        strstr( packet , ";X2DDEBUG;" ) != NULL ||
        strstr( packet , ";RFUDEBUG=" ) != NULL ||
        strstr( packet , ";QRFDEBUG=" ) != NULL
    )
    {
        return true;
    }
    return false;
}


//------------------------------------------------------------------------------
// Filtering the specific devices. See the deviceignorelist vector in
// "yoursettings.h" header file

bool rflinkfilterdevice( char * packet )
{
    int i;

    for( i = 0 ; deviceignorelist[i] ; i++ )
    {
        if( strstr( packet , deviceignorelist[i] ) )
        {
            return true;
        }
    }
    return false;
}

//------------------------------------------------------------------------------
// Handling the raw packet data received via serial link

bool rflinkreadline( String & out )
{
    int available;
    int buffstartpos = serialbuffpos;
    char c;

    if( (available=rflink.available()) )
    {
        ledblink();
        DEBUG_PRINTF( "%d: >>> Received (%d): " , millis() , available );
        if( (serialbuffpos+available) >= sizeof(serialbuff) )
        {
            rprintf("!!! ERROR: buffer overflow (buffer %d)\n" , sizeof(serialbuff) );
            serialbuffpos += available;
            rflinkbuffoverflow = true;
        }

        while( rflink.available() )
        {
            c = rflink.read();
            if( c == '\r' || !c )
            {
                continue;
            }
            else if( c == '\n' )
            {
                int length = serialbuffpos;
                serialbuff[serialbuffpos] = 0;
                serialbuffpos = 0;
                if( rflinkbuffoverflow )
                {
                    SYSLOG( LOG_ERR , "Serialbuffer overflow! Total bytes received: %d", length );
                    debugpacket = false;
                    return false;
                }
                // If something is went wrong...
                if( debugpacket )
                {
                    if( buffstartpos && !strncmp( &serialbuff[buffstartpos] , "20;" , 3 ) )
                    {
                        rprintf("!!! WARNING: Packet collision! Dropping the unfinished debug packet!\n" );
                        // Drop the early buffer content
                        memcpy( serialbuff , &serialbuff[buffstartpos] , strlen(&serialbuff[buffstartpos])+1 );
                        buffstartpos = 0;
                        DEBUG_PRINTF( "=> Overlapped packet: " );
                    }
                    else
                    {
                        DEBUG_PRINTF( "=> Debug packet - end\n" );
                    }
                    debugpacket = false;
                }
                // FIXME: debug packet too long for print....
                if( !buffstartpos )
                {
                    DEBUG_PRINTF("%s\n" , &serialbuff[buffstartpos] );
                }
                // Checking the device filters
                if( rflinkfilterdevice( serialbuff ) == true )
                {
                    // Drop the filtered devices packet
                    rprintf("!!! WARNING: Packet filtered: \"%s\"!\n" , serialbuff );
                    rfpartialpackage = 0;
                    rflinkbuffoverflow = false;
                    return false;
                }

                if( strncmp( serialbuff , "20;" , 3 ) )
                {
                    rprintf("!!! ERROR: Invalid packet received!\n" );
                    rprintf(">>> PACKET: %s (first char: 0x%x)\n" , serialbuff , (int)serialbuff[0] );

                    // Sometime just the first character missing or that is garbage
                    // In this case try to fix the block
                    if( serialbuff[0] == '0' &&
                        serialbuff[1] == ';' &&
                        serialbuff[4] == ';'
                        // Maybe lso check the last character
                        // && serialbuff[strlen(serialbuff)-1] == ";"
                      )
                    {
                        if( strlen(serialbuff) < sizeof(serialbuff)-2 )
                        {
                            rprintf("!!! WARNING: Repair the block!\n" );
                            for( int i = strlen(serialbuff) ; i >= 0 ; i-- )
                            {
                                serialbuff[i+1] = serialbuff[i];
                            }
                            serialbuff[0]='2';
                            rfpartialpackage = 0;
                            rprintf(">>> Repaired block: %s\n" , serialbuff );
                            SYSLOG( LOG_ERR , "Invalid packet repaired!" );
                        }
                        else
                        {
                            rprintf("!!! ERROR: The data is too long in buffer, cannot repair!\n" );
                        }
                    }
                    else if( strncmp( &serialbuff[1] , "20;" , 3 ) == 0 &&
                            // The 49 beta is restart very often (and in this case the message
                            // start with an extra character) which is generate
                            // unnecessary version requests so shold be filter this case
                            ( strstr( serialbuff , RFLINK_WELCOME ) == NULL ||
                              strstr( serialbuff , "R49" ) == NULL
                            )
                    )
                    {
                        rprintf("!!! WARNING: Repair the block!\n" );
                        for( int i = 0 ; i <= strlen(serialbuff); i++ )
                        {
                            serialbuff[i] = serialbuff[i+1];
                        }
                        rfpartialpackage = 0;
                        rprintf(">>> Repaired block: %s\n" , serialbuff );
                        SYSLOG( LOG_ERR , "Invalid packet repaired!" );
                    }
                    else
                    {
                        rprintf( "Invalid packet received!" );
                        SYSLOG( LOG_ERR , "Invalid packet received!" );
                        rfpartialpackage++;
                        return false;
                    }
                }
                else if( !buffstartpos )
                {
                    rfpartialpackage = 0;
                }

                // 20;89;OK;
                if( serialbuff[0] == '2' && serialbuff[1] == '0' && serialbuff[2] == ';' &&
                    serialbuff[5] == ';' &&
                    serialbuff[6] == 'O' && serialbuff[7] == 'K'
                )
                {
                    if( rflinkpendingcommand == true )
                    {
                        rprintf("--> Clearing pending command state...\n" );
                        rflinkpendingcommand = false;
                    }
                }

                rflinkbuffoverflow = false;
                out = (String)serialbuff;
                return true;
            }
            else if( rflinkbuffoverflow == false )
            {
                serialbuff[serialbuffpos++] = c;
                //DEBUG_PRINTF( "%c" , c );
            }
        }
    }
    bool status = false;

    serialbuff[serialbuffpos] = 0;
    if( !buffstartpos && rflinkisdebugpacket( serialbuff ))
    {
        debugpacket = true;
    }

    if( !debugpacket )
    {
        // Welcome message is not contains RETURN
        if( strstr( serialbuff , RFLINK_WELCOME ) == NULL )
        {
            DEBUG_PRINTF( "=> Packet do not contains RETURN caracter...\n" );
            rfpartialpackage++;
        }
        else
        {
            status = true;
        }
    }
    else
    {
        DEBUG_PRINTF( "=> Debug packet..\n" );
    }
    DEBUG_PRINTF("%s\n" , &serialbuff[buffstartpos] );
    return status;
}

//------------------------------------------------------------------------------
// The function is split the rflink packet to separeted fields
// Example: 20;BC;Oregon TempHygro;ID=428AA;TEMP=00d5;HUM=33;HSTATUS=2;BAT=OK;

size_t rflinksplitpacket( char * packet , const char * separator, char ** pfields , size_t maxfields , size_t *length = NULL )
{
    char        *s = packet;
    char        *p,*end;
    int         fields = 0;
    size_t      len = 0;

    // DEBUG: Just for test
    bool debugsave = SYSTEMdebug;
    // SYSTEMdebug = true;

    end = &packet[strlen(packet)];
    //fields = StringSplit( (String)t , ';' , params , 5 );
    DEBUG_PRINTF("Splitting fields: " );
    p = strstr( s , separator );
    if( p != NULL )
    {
        for( int i = 0; s != NULL && s < end && strlen( s ); i++ )
        {
            if( fields < maxfields )
            {
                pfields[fields] = s;
            }
            fields++;
            if( p != NULL )
            {
                *p = 0;
            }
            len += strlen(s);
            DEBUG_PRINTF(" \"%s\"" , s );
            s = p;
            if( s != NULL )
            {
                s++;
                p = strstr( s , separator );
            }
        }
    }
    else
    {
        // If there is no separator at all
        pfields[0] = s;
        len = strlen(s);
    }

    DEBUG_PRINTF("\n" );
    if( length != NULL )
    {
        *length = len;
    }

    // DEBUG: Just for test
    SYSTEMdebug = debugsave;

    return fields;
}

//------------------------------------------------------------------------------
// Sending master broadcast over MQTT for letting known all other RFLINKL
// nodeswho is the main node here because One node to rule them all

#ifdef MULTIGATEWAY

unsigned long  broadcastsent = 0;

void rflinkmasterbroadcast( void )
{
    // WARNING!!!: if you cahnege this condition keep in mind the recursive
    // call in some lines below
    if( rflinkmastermode )
    {
        StaticJsonBuffer<200> jsonBuffer;
        String topic;
        String payload;

        topic = String(MQTT_CONTROLLERCOMMANDS);
        JsonObject& root = jsonBuffer.createObject();
        root["command"] = "mqttproxystatus";
        root["value"] = "master";
        root["ip"] = WiFi.localIP().toString();
        root["name"] = esp32devicename;
        root.printTo( payload );
        // payload.replace( "\\r\\n" , "" );
        MQTTPublish( topic , payload );
        rfbroadcast = millis();
    }
    else
    {
        // Discover the master node if it is present
        rprintf( "%s: >>>> Last master heartbit: %ld\n", timeClient.getFormattedTime().c_str() , ( millis() - rflinkmasterheartbit ) );

        if( ( millis() - rflinkmasterheartbit ) > 60000 )
        {
            if( broadcastsent != 0 && ( millis() - broadcastsent ) < 60000 )
            {
                // Randomise the time to avoid tthe multiple master after a blackout
                if( ( millis() - broadcastsent ) > ( 7000 + rfbroadcastrnd ) &&
                    rflinkknownmaster == false
                )
                {
                    rprintf( "%s: >>>> No master detected in the system! I am promote myself!\n", timeClient.getFormattedTime().c_str() );
                    // If the request is already sent and there is no answare we are alone...
                    rflinkmastermode = true;
                    rflinkknownmaster = true;
                    // WARNING!!! This is recursive call
                    // Send the master parameters instantly
                    rflinkmasterbroadcast();
                }
            }
            else
            {
                StaticJsonBuffer<200> jsonBuffer;
                String topic;
                String payload;
                // Send discover request
                topic = String(MQTT_CONTROLLERCOMMANDS);
                JsonObject& root = jsonBuffer.createObject();
                root["command"] = "discovermqttproxy";
                root["ip"] = WiFi.localIP().toString();
                root["name"] = esp32devicename;
                root.printTo( payload );
                // payload.replace( "\\r\\n" , "" );
                MQTTPublish( topic , payload );
                broadcastsent = millis();
                rflinkknownmaster = false;
                rfbroadcast = millis();
            }
        }
    }
}

#endif // MULTIGATEWAY

//------------------------------------------------------------------------------
// Node specific MQTT comands

void mqttcallback(char* topic, uint8_t * payload, unsigned int length)
{
    // if( MQTTtopicmatch( topic , RFLINK_RECEIVE_TOPIC ) )
    {
        rprintf(">>> Message arrived [%s]: \n" , payload );
        StaticJsonBuffer<255> json;
        JsonObject& root = json.parseObject((const char*)payload);
        if( root.success() )
        {
#ifdef MULTIGATEWAY
            if( root["command"].as<String>() == "mqttproxystatus" )
            {
                if( root.containsKey("command") && root["value"].as<String>() == "master" )
                {
                    rflinkknownmaster = true;
                    rflinkmastermode = true;
                }
                else if( root.containsKey("command") && root["value"].as<String>() == "slave" )
                {
                    rflinkknownmaster = false;
                    rflinkmastermode = false;
                    rfbroadcast = millis();
                }
            }
#endif // MULTIGATEWAY
        }
        else
        {
            rprintf( "!!! ERROR: Cannot parse payload as JSON!\n" );
        }
    }
}

// WakeOnLan command handling

void mqttwoltopic(char* topic, uint8_t * payload, unsigned int length)
{
    StaticJsonBuffer<256> json;

    rprintf( "%s: >>> WOL command: %s\n", timeClient.getFormattedTime().c_str() , (const char*)payload );
    JsonObject& root = json.parseObject((const char*)payload);
    if( root.success() && root.containsKey("mac") )
    {
        WOL target;
        target.wol( root["mac"].as<const char *>() );
    }
    else
    {
        rprintf( "!!! ERROR: invalid payload: %s\n" , payload );
    }
}

// Queue rflink packet for sending

void mqttrflinkouttopic(char* topic, uint8_t * payload, unsigned int length)
{
    // Do not handle the packet sending here just put into handling queue
    // for async handling

    // return _mqttrflinkouttopic( topic, payload, length);

#ifdef MULTIGATEWAY
    // If system is slave avoid to multiple command send
    if( rflinkmastermode == false
        // JUST for DEBUG!!!!
        // || true
        )
    {
        return;
    }
#endif // MULTIGATEWAY

    if( mutex_rflinkqueue != NULL )
    {
        xSemaphoreTakeRecursive( mutex_rflinkqueue, portMAX_DELAY );
    }

    RFLINKqueueitem * message = new RFLINKqueueitem( (const char *)topic , (const char *)payload );
    // Add topic to list
    RFLINKqueue.push_back( message );

    if( mutex_rflinkqueue != NULL )
    {
        xSemaphoreGiveRecursive( mutex_rflinkqueue );
    }

}

// Sending packet over serial

bool _mqttrflinkouttopic(char* topic, uint8_t * payload, unsigned int length)
{
    char        * p;
    size_t      len = 0;
    size_t      sent = 0;
    char        payloadstr[300];
    char        rflinkpacket[300];
    bool        syncback = false;

#ifdef MULTIGATEWAY
    // If system is slave avoid to multiple command send
    // Check this in the queuing functions also
    if( rflinkmastermode == false
        // JUST DEBUG!!!!
        // || true
        )
    {
        // Avoid to multiple command send
        return true;
    }
#endif // MULTIGATEWAY

    if (length < 5 )
    {
        rprintf( "!!! ERROR: Invalid payload, processing skipped!\n");
        return true;
    }

    if ( (length+1) > sizeof(payloadstr) || (length+1) > sizeof(rflinkpacket) )
    {
        rprintf( "!!! ERROR: Payload is too long to handle, skipped!\n");
        return true;
    }

    int i = length>sizeof(payloadstr)-1 ? sizeof(payloadstr)-2 : length;
    // Create C (0 terminated) string
    memcpy( payloadstr , payload , i );
    // Drop the CR/LF
    for( int j = 0; j < 2; j++ )
    {
        if( payloadstr[i-1] == '\n' ) i--;
        if( payloadstr[i-1] == '\r' ) i--;
    }
    payloadstr[i]=0;

    // Skip the version request
    // if( strncmp( payloadstr , "10;VERSION;" , 11 ) == 0 )
    // {
    //     rprintf( "!!! WARNING: Version request is skipped!\n");
    //     return;
    // }

    // Check the syncback necessity
    len = length;
    if( (p = strstr( payloadstr , "SYNCID=" )) != NULL )
    {
        len = (p-payloadstr); //-1;
        syncback = true;
    }
    // TODO: this approach is dirty MUST refoactor using
    // a queue to reschedule the command
    // If the devices or the communication was reseted
    if(rflinklastreset() < 2500 )
    {
//        // delay( 3000 );
        rprintf( "%s: >>> RFLINK command deleayed\n" , timeClient.getFormattedTime().c_str() );
        SYSLOG( LOG_ERR , "RFLINK command deleayed because rflink reset..." );
        return false;
    }

    // Remove the SYNC ID
    memcpy( rflinkpacket , payload , len );
    rflinkpacket[len]=0;
    //sent = rflink.println( rflinkpacket );
    //rflink.flush();
    sent = rflinksend( rflinkpacket );

    // Debug just here to minimalise the packet transfer delay
    rprintf( "%s (%u): >>> RFLINK command received: \"%s\"\n" , timeClient.getFormattedTime().c_str() , millis() , payloadstr );

    // Try to resend if communication error occurred
    if( sent < len+2 )
    {
        rprintf( "%s: >>> !!! ERROR: Packet send error! Packet length %d sent %d\n" , timeClient.getFormattedTime().c_str() , len , sent );
        // FIXME a parameter átadásos syslog hvás nem működik
        char msg[256];
        sprintf( msg , "Packet send error! Packet length %d sent %d\n" , len , sent );
        SYSLOG( LOG_ERR , msg );

        return false;
        // // Resend the packet
        // for( int i = 0 ; i < 3 ; i++ )
        // {
        //     delay( 1000 );
        //     rprintf( "%s: >>> Packet resend... \"%s\"\n" , timeClient.getFormattedTime().c_str() , rflinkpacket );
        //     sent = rflinksend( rflinkpacket );
        //     //sent = rflink.println( (char *)rflinkpacket );
        //     //rflink.flush();
        //     if( sent == len+2 )
        //     {
        //         break;
        //     }
        // }
    }

    // // FIXME: just test
    // if( DEBUG_MODE() && len != length )
    // {
    //     char    test[256];
    //     memcpy( test , payload , len );
    //     test[len]=0;
    //     DEBUG_PRINTF("Stripped payload: %s\n" , test );
    // }

    // Post back the command in the in channel to sync with other hosts
    char *params[MAX_RFPACKET_FIELDS];
    if( syncback == true &&
        rflinksplitpacket( payloadstr , ";", params , MAX_RFPACKET_FIELDS , &len ) == MAX_RFPACKET_FIELDS &&
        len < (sizeof(rflinkpacket)-24)
    )
    {
        sprintf( rflinkpacket , "20;00;%s;ID=%s;SWITCH=%s;CMD=%s;%s;", params[1] , params[2] ,params[3] ,params[4] , params[5] );
        MQTTPublish( RFLINK_RECEIVE_TOPIC , rflinkpacket );
    }
    else
    {
        rprintf( "%s: !!! WARNING: Unknown command, MQTT postback is SKIPPED!\n" , timeClient.getFormattedTime().c_str() );
    }

    return true;
}

// Handling the a controller specific commands
// FIXME:
// Create the controller topic for
// sending commands to the gateway
// RESET
// DEBUG
// SERIALRESET
// ...
// MQTT TARGET, PORT

void mqttcontrollertopic(char* topic, uint8_t * payload, unsigned int length)
{
    // if( strcmp( t , "10;REBOOT;" ) == 0 )
    // {
    //     // FIXME: Move this to tthe controller topic
    //     rprintf( ">>> RFLINK reset the serial connaction\n" );
    //     rflinkreset();
    // }

    DEBUG_PRINTF( "Controller topic received...\n");

    StaticJsonBuffer<255> json;
    JsonObject& root = json.parseObject((const char*)payload);
    if( root.success() )
    {
        // Filter our own messages
        if( root.containsKey("name") &&
            root["name"].as<String>() == String(esp32devicename)
        )
        {
            DEBUG_PRINTF( "-->> Filter our own message...\n");
            return;
        }

        // Query master nodes
        if( root.containsKey("command") )
        {
#ifdef MULTIGATEWAY
            if( root["command"].as<String>() == "discovermqttproxy" )
            {
                if( rflinkmastermode )
                {
                    // I am the Master
                    rflinkmasterbroadcast();
                }
            }
            else if( root["command"].as<String>() == "mqttproxystatus" )
            {
                if( root.containsKey("command") && root["value"].as<String>() == "master" )
                {
                    String name;

                    rflinkknownmaster = true;
                    if( root.containsKey("ip") )
                    {
                        name = root["ip"].as<String>();
                    }
                    else
                    {
                        name = root["name"].as<String>();
                    }

                    rprintf( "%s: Master heartbit received: %s\n", timeClient.getFormattedTime().c_str() , name.c_str());
                    rflinkmasterheartbit = millis();
                    rfbroadcast = millis();
                    if( rflinkmastermode )
                    {
                        // If somebody else also know himself master
                        // we drop here the status
                        rprintf( "%s: !!! WARNING: Multiple master on the network! I am resign NOW!\n", timeClient.getFormattedTime().c_str());
                        rflinkmastermode = false;
                    }
                }
            }
#endif // MULTIGATEWAY
        }
    }
    else
    {
        rprintf( "!!! ERROR: Cannot parse payload as JSON!\n" );
    }

}

//------------------------------------------------------------------------------
// Handling the received packet

void processmessage( String & message )
{
// Full packet avalible
bool debugpacket;

    // Switch back the RFDEBUG
    // if( message.indexOf( RFLINK_WELCOME ) != -1 ||
    //     (message.indexOf( ";VER=" ) != -1 && message.indexOf( ";REV=" ) != -1 )
    // )
    // {
    //     // Switch on the undecoded packet debugging
    //     //rflinksend( "10;RFUDEBUG=ON;\n");
    //     //DEBUG_PRINTF("%s: Unhandled packet debug set to ON.\n" , timeClient.getFormattedTime().c_str() );
    //     //rprintf("%s: Unhandled packet debug set to ON.\n" , timeClient.getFormattedTime().c_str() );
    // }

    // Skip the special messages
    debugpacket = rflinkisdebugpacket( message.c_str() );

#ifdef ACURITE899CONVERSION
    // RC49 firmware: Originally I implemented the sensor wwith tick count
    // so It should to convert from mm to tick count
    // But I was implemented a separate Acurite 433 decoder because the delayed
    // RFLINK firmware release so this code is technically obsolote...
    if( message.indexOf( ";Acurite899;" ) != -1 )
    {
        DEBUG_PRINTF("%s: Acurite899 precessing...\n" , timeClient.getFormattedTime().c_str() , rfpartialpackage );
        int rpos,rend;
        unsigned long rainval;

        if( (rpos = message.indexOf( ";RAIN=" )) != -1 )
        {
            if( (rend = message.indexOf( ";" , rpos+6 )) != -1 )
            {
                String  str = message.substring( 0 , rpos + 6 );
                rainval = strtol( message.substring( rpos + 6 , rend ).c_str() , 0L , 16);
                DEBUG_PRINTF("Acurite899 rainval: %s Converted: %d " , message.substring( rpos + 6 , rend ).c_str(), rainval );
                rainval = (rainval<<2)/10;
                str += String( rainval , HEX );
                str.concat( message.substring( rend ) );
                message = str;
                DEBUG_PRINTF("rebuilded packet: %s\n" , message.c_str() );
            }
        }
        else
        {
            rprintf("%s: !!! ERROR: Unable to locate rain data in packet!\n" , timeClient.getFormattedTime().c_str());
        }
    }
#endif // ACURITE899CONVERSION

    // Consume debug packets
    if( debugpacket == true ||
        message.indexOf( ";PONG;" ) != -1
    )
    {
        message = "";
    }
    // There is any message?
    if( message.length() )
    {
        //rprintf( "Message to publish: %s\n" , message.c_str() );
        // Publish mqtt message
        if( message.startsWith( "20;" ) )
        {
            MQTTPublish( (String)RFLINK_RECEIVE_TOPIC , message );
        }
        else
        {
            rprintf("%s: !!! ERROR: Invalid packet, SKIPPED: %s\n" , timeClient.getFormattedTime().c_str() , message.c_str());
            // FIXME: use of parameters in syslog call does not work...
            char msg[512];
            if( message.length() < 450 )
            {
                sprintf( msg , "Invalid packet, SKIPPED: %s" , message.c_str() );
            }
            else
            {
                sprintf( msg , "Invalid packet, SKIPPED: (content is too long to display)" );
            }
            SYSLOG( LOG_ERR , msg );
        }
    }
}

//------------------------------------------------------------------------------
// OTA start callback

void otastart( void )
{
    rprintf( ">>> OTA start disable all function...");
    disablewatchdog();
    rflinkend();
    MQTTend();
}

//------------------------------------------------------------------------------
// Init sequence

void setup() {

    Serial.begin(BAUD);                                    // Initialise the serial port
    WifiConnect( MY_WIFI_AP , MY_WIFI_PSW );
    sprintf( esp32devicename , "esp32-rflink-%s" , getMacAddress( true ).c_str());
    utilsinit();
    rprintf("Setup started...\n");
    rprintf("MAC address: %s\n", getMacAddress( true ).c_str());

    mutex_rflinkqueue = xSemaphoreCreateRecursiveMutex( );

    rprintf( "CPU speed: %d\n" , getCpuFrequencyMhz());

    ledblink( LEDBLINK_INIT );

    //pinMode(DATAPIN, INPUT);
    //https://github.com/espressif/arduino-esp32/blob/master/tools/sdk/include/driver/driver/gpio.h

    rprintf("OTA Init...\n");
    OTAinit( 3232 , esp32devicename , NULL , otastart );
    Serial.printf("Step 6\n");

    // Connect to the MQTT broker
    rprintf("MQTT Init...\n");
#ifndef MQTT_BROKER
    MQTTConnect( esp32devicename , MQTT_USER , MQTT_PSW );
#else
    MQTTConnect( MQTT_BROKER , MQTT_PORT , esp32devicename , MQTT_USER , MQTT_PSW );
#endif
    Serial.printf("Step 1\n");

    // Send a Welcome message just for fun
    String message = "Hello from " + String(esp32devicename);
    MQTTPublish( TEST_TOPIC , message );

    // Subscribe for the default topics
    MQTTSubscribe( mqttcontrollertopic );
    MQTTSubscribe( RFLINK_CONTROL_TOPIC , mqttrflinkouttopic );
    MQTTSubscribe( "wol" , mqttwoltopic );

    // Init RFLink connection
    Serial.println( "Serial init..." );
    rflinkinit();
    rprintf("Reboot rflink..\n");
    rflinksend( "10;REBOOT;\n");

    // Init the system watchdog witch will reset the ESP32 if something was went wrong
    initwatchdog();

    // Init randomised timeshift for master slave mode
    randomSeed( millis());
    rfbroadcastrnd = random( 555 , 5555 ) << 1;
    rfbroadcast = millis() + rfbroadcastrnd;
    rprintf( "Broadcast shift value: %ld\n" , rfbroadcastrnd );

    rprintf("Setup is finished\n");
    SYSLOG( LOG_ERR , "RFLINK MQTT gateway is started..." );
}

//------------------------------------------------------------------------------
// Main loop

void loop()
{

    //----------------------------------------------------------------------
    // Generic background proocesses...

    WifiConnect();
    OTAHandle();
    utilsloop();

    //--------------------------------------------------------------------------
    // Handling the data imput from the RFLINK device

    if( rflinkdataavailable() )
    {
        while( rflinkdataavailable() )
        {
            if( rflinkreadline(serialimput) )
            {
                rflastreceived = millis();
                //rprintf( "%d: >>> Buffer closed: %s\n" , millis() , serialimput.c_str() );
                processmessage( serialimput );
            }
            if( rfpartialpackage )
            {
                DEBUG_PRINTF("%s: Partial packages received: %d\n" , timeClient.getFormattedTime().c_str() , rfpartialpackage );
            }
        }
    }

    //----------------------------------------------------------------------
    // Handling the sending QUEUE

    if( RFLINKqueue.size() )
    {
        rprintf("%s: Items in out queue: %d\n" , timeClient.getFormattedTime().c_str(),RFLINKqueue.size());

        if( mutex_rflinkqueue != NULL )
        {
            xSemaphoreTakeRecursive( mutex_rflinkqueue, portMAX_DELAY );
        }

        std::vector<RFLINKqueueitem*>::iterator it = RFLINKqueue.begin();

        while ( it != RFLINKqueue.end() )
        {
            // rprintf( "Handle rflink out topic.\n" );
            if( _mqttrflinkouttopic( (char *)(*it)->topic.c_str() , (uint8_t *)(*it)->payload.c_str() , (*it)->payload.length() ) )
            {
                // rprintf( "Delete object!\n" );
                delete *it;
                // rprintf( "Delete object from list!\n" );
                it = RFLINKqueue.erase( it );
                rprintf( "Command succesfully handled! Delete object from list!\n" );
                // delay( 25 );
            }
            else
            {
                rprintf( "Packet handle failed! Retry count: %d\n" , (*it)->retry  );
                if( ++(*it)->retry > 25 )
                {
                    rprintf( "Packet send continously failed! Delete rflink command object.\n" );
                    delete *it;
                    it = RFLINKqueue.erase( it );
                }
            }
        }

        if( mutex_rflinkqueue != NULL )
        {
            xSemaphoreGiveRecursive( mutex_rflinkqueue );
        }
    }
    else
    {
        delay( 25 );
    }

    //----------------------------------------------------------------------
    // Send master broadcast
    // if you use multiple RFLINK MQTT gateway

#ifdef MULTIGATEWAY
    if( ( rfbroadcastinitdelay == true && millis() > rfbroadcast ) ||
        ( rfbroadcastinitdelay == false && (millis() - rfbroadcast) >
            ( (rflinkmastermode == false && rflinkknownmaster == false ) ? 5000: 30000+rfbroadcastrnd)
        )
    )
    {
        rprintf("%s: --- RFLink %s ---\n" , timeClient.getFormattedTime().c_str(),
            rflinkmastermode ? "Master broadcast" : "discover master node"
        );
        rflinkmasterbroadcast( );
        rfbroadcast = millis();
        rfbroadcastinitdelay = false;
    }
#endif

    //----------------------------------------------------------------------
    // Heartbit

    if( rflinklastsent() > 15000 && ( millis() - rfheratbit) > 60000 && !rflink.available() )
    {
        rprintf("%s: --- RFLink Heartbit ---\n" , timeClient.getFormattedTime().c_str());
        rflinksend( "10;PING;\n");
        rfheratbit = millis();
    }

    //----------------------------------------------------------------------
    // Try to reset RFLINK device

    if( rflinklastsent() > 15000 && (  millis() - rflastreceived ) > 900000 )
    {
        SYSLOG( LOG_ERR , "Rebooting RFLink device." );
        rprintf("%s: !!! Try to rebbot rflink device...\n" , timeClient.getFormattedTime().c_str());
        rflinkreset();
        rflinksend( "10;REBOOT;\n");
        //delay( 500 );
        serialimput = "";
        rflastreceived = millis();
        rfpartialpackage = 0;
    }

    //----------------------------------------------------------------------
    // Heartbit

    if( rflinklastsent() > 15000 && rfpartialpackage > 3 )
    {
        SYSLOG( LOG_ERR , "Reseting serial communication." );
        rprintf("%s: !!! Try to reset serial communication...\n" , timeClient.getFormattedTime().c_str());
        rflinkreset();
        serialimput = "";
        rflastreceived = millis();
        rfpartialpackage = 0;
    }
}
