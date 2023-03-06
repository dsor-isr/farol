#!/usr/bin/env python
import sys
import signal
import os
import socket
import traceback
import genpy

import yaml
import time

# Threads
import threading

# ROS stuff
import roslib.message
import rospy
import rosgraph

# HTTP Server
import cgi, cgitb, urllib
#from urlparse import urlparse
from future.standard_library import install_aliases
install_aliases()

from urllib.parse import urlparse, urlencode
from urllib.request import urlopen, Request
from urllib.error import HTTPError

#from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from http.server import HTTPServer, BaseHTTPRequestHandler

from socketserver import ThreadingMixIn
#from SocketServer import ThreadingMixIn
from _socket import error as SocketError

import array
NAME='Console'
MISSION_PATH ="/home/cog/cog-sw/Missions_FOLDER" # TODO: Change this to a PARAMETER
#MISSION_PATH ="/home/jorge/Farol_CODE/Missions_FOLDER" # TODO: Change this to a PARAMETER
pages_folder = ""
NFILE=0 # Temp filename

from operator import itemgetter

# Services
from waypoint.srv import *

class ROSTopicException(Exception):
    """
    Base exception class of rostopic-related errors
    """
    pass
class ROSTopicIOException(ROSTopicException):
    """
    rostopic errors related to network I/O failures
    """
    pass
    
class Topic_Struct(object):
    # TODO: Include Mutexs to this struct
    def __init__(self):
        self.time_rcv = []
        self.topics = []
        self.topics_data = []
    
    # Print all the topics to a string
    def __str__(self):
        text = ""
        for i in range(len(self.topics)):
            tnow = rospy.Time.now()
            if((tnow-self.time_rcv[i]).to_sec() < 10):
                print ("---->" + text)
                text += ros2xml(self.topics_data[i], self.topics[i],0)
        return text
    
    # Add a topic or substitute one
    def add_value(self, topic_name, data):
        tnow = rospy.Time.now()
        if not topic_name.startswith("/"):
            topic_name = "/" + topic_name
        if self.topics.count(topic_name) == 0 : # new topic
            self.topics.append(topic_name)
            self.topics_data.append(data)
            self.time_rcv.append(tnow)
        else: # update existing
            self.topics_data[self.topics.index(topic_name)] = data
            self.time_rcv[self.topics.index(topic_name)] = tnow

    
    # Add a topic or substitute one
    def del_value(self, topic_name):
        if self.topics.count(topic_name) == 0 :
            return
        ind = self.topics.index(topic_name)
        self.topics.pop(ind)
        self.topics_data.pop(ind)
        self.time_rcv.pop(ind)

    # Search for a topic name
    def data_topic(self, topic_name):
        tnow = rospy.Time.now()
        if not topic_name.startswith("/"):
            topic_name = "/" + topic_name
        if self.count(topic_name) > 0 :
            if((tnow-self.time_rcv[self.topics.index(topic_name)]).to_sec() < 10):
                return [self.topics_data[self.topics.index(topic_name)], self.time_rcv[self.topics.index(topic_name)]]
        return [0, 0]
    
    # Number of times that a topic name appears
    def count(self, topic_name):
        if not topic_name.startswith("/"):
            topic_name = "/" + topic_name
        #print ("count:" + topic_name + " " + str(self.topics.count(topic_name)))
        #print (self.topics)
        count_topic = self.topics.count(topic_name)
        tnow = rospy.Time.now()
        # if(count_topic>0):
        #     if((tnow-self.time_rcv[self.topics.index(topic_name)]).to_sec() >= 10):
        #         return 0
        return count_topic
        
    # Returns a String with all the topics saved
    def str_list(self):
        msg = ""
        for i in self.topics:
            msg += i + "\n"
        return msg
ALL_TOPICS = Topic_Struct()
SUBSCRIBED_TOPICS = Topic_Struct()
UNKNOWN_TOPICS = Topic_Struct()

# ERRORS List for rosout
class ERROR_Struct(object):
    # TODO: Include Mutexs to this struct
    def __init__(self):
        self.node_name = []
        self.line = []
        self.msg = []
        self.time = []
        self.time_rcv = []
    
    def __clear__(self):
        self.node_name = []
        self.line = []
        self.msg = []
        self.time = []
        self.time_rcv = []
    # 
    def __clearitem__(self, i):
        tnow = rospy.Time.now()
        if( self.time[i]!=-1 and ( tnow - self.time[i]).to_sec() > 1):
            self.node_name.pop(i)
            self.line.pop(i)
            self.msg.pop(i)
            self.time.pop(i)
            self.time_rcv.pop(i)
            return True
        elif ( self.time[i] == -1):
            self.time[i] = tnow
            return False
        return False

    # Print all the error messages to a string and deletes everything
    def __str__(self):
        if len(self.node_name)==0:
            return ""
        # self.__clear__()
        text="\t<ROSMessage name=\"Errors\">"
        # do the range from the end to the beginning
        for i in range(len(self.node_name)-1,-1,-1):
            if self.__clearitem__(i):
                continue
            text += "\t\t<VAR>\n\t\t\t<NODE>"+self.node_name[i]+"</NODE>\n"
            text += "\t\t\t<TIME>"+"%.2f"%(self.time_rcv[i].to_sec())+"</TIME>\n"
            text += "\t\t\t<MSG>"+self.msg[i]+"</MSG>\n\t\t</VAR>"
        text+="</ROSMessage>"
        #print text
        return text
    
    # Add an error message to the list
    def add_error(self, topic_name, data):
        if topic_name.endswith("rosout"):
            if(int(data.level)>=8): # Only Error and Fatal messages
                for i in range(len(self.node_name)):
                    if(data.name==self.node_name and data.line==self.line):
                        return
                self.node_name.append(data.name)
                self.msg.append(data.msg)
                self.line.append(data.line)
                self.time.append(-1)
                self.time_rcv.append(rospy.Time.now())

ALL_ERRORs = ERROR_Struct()
       
# ROS Sleep
def _sleep(duration):
    #rospy.rostime.wallsleep(duration)
    rospy.sleep(duration)


def get_topic_class(topic, blocking=False):
    """
    Get the topic message class
    :returns: message class for topic, real topic
      name, and function for evaluating message objects into the subtopic
      (or ``None``). ``(Message, str, str)``
    :raises: :exc:`ROSTopicException` If topic type cannot be determined or loaded
    """
    topic_type, real_topic, msg_eval = get_topic_type(topic, blocking=blocking)
    if topic_type is None:
        return None, None, None
    msg_class = roslib.message.get_message_class(topic_type)
    if not msg_class:
        raise ROSTopicException("Cannot load message class for [%s]. Are your messages built?"%topic_type)
    return msg_class, real_topic, msg_eval

def get_topic_type(topic, blocking=False):
    """
    Get the topic type.

    :param topic: topic name, ``str``
    :param blocking: (default False) block until topic becomes available, ``bool``
    
    :returns: topic type, real topic name and fn to evaluate the message instance
      if the topic points to a field within a topic, e.g. /rosout/msg. fn is None otherwise. ``(str, str, fn)``
    :raises: :exc:`ROSTopicException` If master cannot be contacted
    """
    topic_type, real_topic, msg_eval = _get_topic_type(topic)
    if topic_type:
        return topic_type, real_topic, msg_eval
    elif blocking:
        sys.stderr.write("WARNING: topic [%s] does not appear to be published yet\n"%topic)
        while not rospy.is_shutdown():
            topic_type, real_topic, msg_eval = _get_topic_type(topic)
            if topic_type:
                return topic_type, real_topic, msg_eval
            else:
                _sleep(0.1)
    return None, None, None

def _get_topic_type(topic):
    """
    subroutine for getting the topic type
    :returns: topic type, real topic name and fn to evaluate the message instance
    if the topic points to a field within a topic, e.g. /rosout/msg, ``(str, str, fn)``
    """
    try:
        val = _master_get_topic_types(rosgraph.Master('/rostopic'))
    except socket.error:
        raise ROSTopicIOException("Unable to communicate with master!")

    # exact match first, followed by prefix match
    matches = [(t, t_type) for t, t_type in val if t == topic]
    if not matches:
        matches = [(t, t_type) for t, t_type in val if topic.startswith(t+'/')]
        # choose longest match
        matches.sort(key=itemgetter(0), reverse=True)
    if matches:
        t, t_type = matches[0]
        if t_type == rosgraph.names.ANYTYPE:
            return None, None, None
        return t_type, t, msgevalgen(topic[len(t):])
    else:
        return None, None, None

# NOTE: this is used externally by rxplot

def _master_get_topic_types(master):
    try:
        val = master.getTopicTypes()
    except xmlrpclib.Fault:
        #TODO: remove, this is for 1.1
        sys.stderr.write("WARNING: rostopic is being used against an older version of ROS/roscore\n")
        val = master.getPublishedTopics('/')
    return val

def _rostopic_cmd_echo(argv):
    def expr_eval(expr):
        def eval_fn(m):
            return eval(expr)
        return eval_fn

    args = argv[2:]
    if len(args) == 0:
        sys.stderr.write("You have to mention the topic\n")
    elif len(args) == 1:
        topic = rosgraph.names.script_resolve_name('rostopic', args[0])
        # suppressing output to keep it clean
        #if not options.plot:
        #    print "rostopic: topic is [%s]"%topic
            
        filter_fn = None
        
        
        callback_echo = CallbackEcho(topic);""", None, False, None, False, False, False, None, None)"""
        try:
            _rostopic_echo(topic, callback_echo)
        except socket.error:
            sys.stderr.write("Network communication failed. Most likely failed to communicate with master.\n")

def msgevalgen(pattern):
    """
    Generates a function that returns the relevant field (aka 'subtopic') of a Message object
    :param pattern: subtopic, e.g. /x. Must have a leading '/' if specified, ``str``
    :returns: function that converts a message into the desired value, ``fn(Message) -> value``
    """
    if not pattern or pattern == '/':
        return None
    def msgeval(msg):
        # I will probably replace this with some less beautiful but more efficient
        try:
            return eval('msg'+'.'.join(pattern.split('/')))
        except AttributeError as e:
            sys.stdout.write("no field named [%s]"%pattern+"\n")
            return None
    return msgeval

def _rostopic_echo(topic, callback_echo):
    """
    Print new messages on topic to screen.
    
    :param topic: topic name, ``str``
    :param bag_file: name of bag file to echo messages from or ``None``, ``str``
    """

    _check_master()
    rospy.init_node(NAME, anonymous=True)
    msg_class, real_topic, msg_eval = get_topic_class(topic, blocking=True)
    if msg_class is None:
        # occurs on ctrl-C
        return
    callback_echo.msg_eval = msg_eval

    use_sim_time = rospy.get_param('/use_sim_time', False)
    sub = rospy.Subscriber(real_topic, msg_class, callback_echo.callback, topic)

    if use_sim_time:
        # #2950: print warning if nothing received for two seconds

        timeout_t = time.time() + 2.
        while time.time() < timeout_t and \
                callback_echo.count == 0 and \
                not rospy.is_shutdown() and \
                not callback_echo.done:
            _sleep(0.1)

        if callback_echo.count == 0 and \
                not rospy.is_shutdown() and \
                not callback_echo.done:
            sys.stderr.write("WARNING: no messages received and simulated time is active.\nIs /clock being published?\n")

    while not rospy.is_shutdown() and not callback_echo.done:
        _sleep(0.1)

def _check_master():
    """
    Make sure that master is available
    :raises: :exc:`ROSTopicException` If unable to successfully communicate with master
    """
    try:
        rosgraph.Master('/rostopic').getPid()
    except socket.error:
        raise ROSTopicIOException("Unable to communicate with master!")
 
class CallbackEcho(object):
    """
    Callback instance that can print callback data in a variety of
    formats. Used for all variants of rostopic echo
    """

    def __init__(self, topic):
        """
        :param plot: if ``True``, echo in plotting-friendly format, ``bool``
        :param filter_fn: function that evaluates to ``True`` if message is to be echo'd, ``fn(topic, msg)``
        :param echo_all_topics: (optional) if ``True``, echo all messages in bag, ``bool``
        :param offset_time: (optional) if ``True``, display time as offset from current time, ``bool``
        :param count: number of messages to echo, ``None`` for infinite, ``int``
        :param field_filter_fn: filter the fields that are strified for Messages, ``fn(Message)->iter(str)``
        """
        if topic and topic[-1] == '/':
            topic = topic[:-1]
        self.topic = topic
        #self.msg_eval = msg_eval
        #self.plot = plot
        #self.filter_fn = filter_fn
        self.count = 0
        self.prefix = ''
        self.suffix = '\n---' #if not plot else ''# same as YAML document separator, bug #3291
        
        #self.echo_all_topics = echo_all_topics
        #self.offset_time = offset_time

        # done tracks when we've exceeded the count
        self.done = False

        # determine which strifying function to use
        #TODOXXX: need to pass in filter function
        self.str_fn = genpy.message.strify_message
        #if echo_clear:
        self.prefix = '\033[2J\033[;H'

        #self.field_filter=field_filter_fn
        
        # first tracks whether or not we've printed anything yet. Need this for printing plot fields.
        self.first = True

        # cache
        #self.last_topic = None
        #self.last_msg_eval = None

    def callback(self, data, topic, current_time=None):
        """
        Callback to pass to rospy.Subscriber or to call
        manually. rospy.Subscriber constructor must also pass in the
        topic name as an additional arg
        :param data: Message
        :param topic: topic name, ``str``
        :param current_time: override calculation of current time, :class:`genpy.Time`
        """
        try:
            if topic == self.topic:
                pass
            elif self.topic.startswith(topic + '/'):
                # self.topic is actually a reference to topic field, generate msgeval
                # generate msg_eval and cache
                self.last_msg_eval = msg_eval = msgevalgen(self.topic[len(topic):])
                self.last_topic = topic
            elif not self.echo_all_topics:
                return
            msg_eval = None
            
            if msg_eval is not None:
                data = msg_eval(data)
            else:
                val = data
                
            # data can be None if msg_eval returns None
            if data is not None:
                # NOTE: we do all prints using direct writes to sys.stdout, which works better with piping
                
                self.count += 1
                
                if topic.endswith('rosout'):
                    ALL_ERRORs.add_error(topic, data)
                else:
                    ALL_TOPICS.add_value(topic, data)
                
                
                #sys.stdout.write(self.prefix+\
                #                ros2xml(data, topic[1:],0) + \
                #                self.suffix + '\n')
                # we have to flush in order before piping to work
                sys.stdout.flush()
            # #2778 : have to check count after incr to set done flag
            #if self.max_count is not None and self.count >= self.max_count:
            #    self.done = True

        except IOError:
            self.done = True
        except:
            # set done flag so we exit
            self.done = True
            traceback.print_exc()

def ros2xml(msg, name, depth=0):
    xml ="";
    tabs = "\t"*depth

    if hasattr(msg, "_type"):
        type = msg._type
        xml = xml + tabs + "<" + name.replace("/", "_") + " type=\"" + type + "\">\n"
        
        try:
            for slot in msg.__slots__:
                xml = xml + ros2xml(getattr(msg, slot), slot, depth=depth+1)
                
        except:
            xml = xml + tabs + str(msg)
        xml = xml + tabs + "</" + name.replace("/", "_") + ">\n"
    else:
        xml = xml + tabs + "<" + name.replace("/", "_") + ">" + str(msg) + "</" + name.replace("/", "_") + ">\n"
    
    return xml
    
def moos2xml(msg, name, time, depth=0):
    xml ="";
    tabs = "\t"*depth

    if hasattr(msg, "_type"):
        xml = xml + tabs + "<ROSMessage name=\"" + name.replace("/", "_") + "\">\n"
        try:
            for slot in msg.__slots__:
                if slot!="stamp":# and slot!="seq":
                    xml = xml + moos2xml(getattr(msg, slot), slot, time, depth=depth+1)
        except:
            pass
        xml = xml + tabs + "</ROSMessage>\n"
    else:
        if name!= "stamp":# and name!= "seq":
            xml = xml + tabs + "<VAR>\n"+ tabs + "\t<KEY>"+ name.replace("/", "_") + "</KEY>\n"
            xml = xml + tabs + "<TIME>\n"+ str(time.to_sec()) + "</TIME>\n"
            xml = xml + tabs + "\t<DOUBLE>"+ str(msg) + "</DOUBLE>\n"+ tabs + "</VAR>\n"
    return xml

def _fullusage():
    print("""rostopic is a command-line tool for printing information about ROS Topics.

Commands:
\tconsole echo\tprint messages to screen
\tconsole list\tlist active topics
\tconsole console\tstarts the http server 

Type rostopic <command> -h for more detailed usage, e.g. 'rostopic echo -h'
""")
    sys.exit(getattr(os, 'EX_USAGE', 1))

def create_publisher(topic_name, topic_type):
    """
    Create rospy.Publisher instance from the string topic name and
    type. This is a powerful method as it allows creation of
    rospy.Publisher and Message instances using the topic and type
    names. This enables more dynamic publishing from Python programs.

    :param topic_name: name of topic, ``str``
    :param topic_type: name of topic type, ``str``
    :param latch: latching topic, ``bool``
    :returns: topic :class:`rospy.Publisher`, :class:`Message` class
    """

    # Save the name without namespace
    final_name = topic_name

    # Get the name with the namespace
    topic_name = rosgraph.names.script_resolve_name(NAME, topic_name)
    try:
        msg_class = roslib.message.get_message_class(topic_type)
    except:
        raise ROSTopicException("invalid topic type: %s"%topic_type)
    if msg_class is None:
        pkg = _resource_name_package(topic_type)
        raise ROSTopicException("invalid message type: %s.\nIf this is a valid message type, perhaps you need to type 'rosmake %s'"%(topic_type, pkg))
    # disable /rosout and /rostime as this causes blips in the pubsub network due to rostopic pub often exiting quickly
    #rospy.init_node(NAME, anonymous=True, disable_rosout=True, disable_rostime=True)

    pub = rospy.Publisher('~/' + final_name, msg_class, latch=True, queue_size=1)
    return pub, msg_class

def _publish_latched(pub, msg):
    """
    Publish and latch message. Subroutine of L{publish_message()}.
    
    :param pub: :class:`rospy.Publisher` instance for topic
    :param msg: message instance to publish
    :param once: if ``True``, publish message once and then exit after sleep interval, ``bool``
    :param verbose: If ``True``, print more verbose output to stdout, ``bool``
    """
    try:
        pub.publish(msg)
    except TypeError as e:
        raise ROSTopicException(str(e))

def publish_message(pub, msg_class, pub_args):
    """
    Create new instance of msg_class, populate with pub_args, and publish. This may
    print output to screen.
    
    :param pub: :class:`rospy.Publisher` instance for topic
    :param msg_class: Message type, ``Class``
    :param pub_args: Arguments to initialize message that is published, ``[val]``
    """
    msg = msg_class()
    try:
        # Populate the message and enable substitution keys for 'now'
        # and 'auto'. There is a corner case here: this logic doesn't
        # work if you're publishing a Header only and wish to use
        # 'auto' with it. This isn't a troubling case, but if we start
        # allowing more keys in the future, it could become an actual
        # use case. It greatly complicates logic because we'll have to
        # do more reasoning over types. to avoid ambiguous cases
        # (e.g. a std_msgs/String type, which only has a single string
        # field).
        
        # allow the use of the 'now' string with timestamps and 'auto' with header
        now = rospy.get_rostime() 
        import std_msgs.msg
        keys = { 'now': now, 'auto': std_msgs.msg.Header(stamp=now) }
        genpy.message.fill_message_args(msg, pub_args, keys=keys)
    except genpy.MessageException as e:
        raise ROSTopicException(str(e)+"\n\nArgs are: [%s]"%genpy.message.get_printable_message_args(msg))
    try:
        _publish_latched(pub, msg)
    except rospy.ROSSerializationException as e:
        import rosmsg
        # we could just print the message definition, but rosmsg is more readable
        raise ROSTopicException("Unable to publish message. One of the fields has an incorrect type:\n"+\
                                "  %s\n\nmsg file:\n%s"%(e, rosmsg.get_msg_text(msg_class._type)))

def cmd_set_topic(args):
    args=args[5:]
    args = args.replace("%27", "")
    args = args.replace("%7B", "{")
    args = args.replace("%7D", "}")
    args = args.replace("%7C", "|")
    args = args.replace("%20", " ")
    args = args.replace("%22", "\"")
    args = args.replace("%23", "#")
    args = args.replace("%09", "\n")
    # args = args.replace("%09", "\r\n")
    args = args.replace("%0D", "\n")
    args = args.replace("%3C", "<")
    args = args.replace("%3E", ">")
    topics = args.split("|")

    if (topics[0].strip().split(' ')[0] == 'Mission_String'):
        print("debug: {}".format(args))

    try:
        topics.remove('')
    except:
        pass
        
    #print("topic: %s"%(topics))
    for k in topics:
        set_args = k.split(" ")
        
        try:
            set_args.remove('')
        except:
            pass 
        try:
            topic_name =set_args[0]
            topic_type =set_args[1]
            args=''
            for i in set_args[2:]:
                #print("ARG: %s"%(args))
                args += i + ' '
            if(args==''):
                continue
        except:
            continue    
        try:
            pub_args = []
            pub_args.append(yaml.load(args))
        except Exception as e:
            print("Argument error: %s"% str(e))
            print(args)
            return 0
        
        # print("Publishing topic: %s type: %s value: %s"%(topic_name, topic_type, pub_args))
        # make sure master is online. we wait until after we've parsed the
        # args to do this so that syntax errors are reported first  
        _check_master()

        if(rospy.get_param("~topics/console/waypoint")  in topic_name):
            try:
                send_wp_standard = rospy.ServiceProxy(topic_name, sendWpType1)
                resp1 = send_wp_standard(pub_args[0]['point']['y'], pub_args[0]['point']['x'], 0)
                return 0
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        else:
            pub, msg_class = create_publisher(topic_name, topic_type)

            try:
                publish_message(pub, msg_class, pub_args)
            except Exception as e:
                print(str(e))
                return 0

    return 1
        
        #print("number of connections %d"%pub.get_num_connections())
        # Wait for Subscribers
        #timeout_t = time.time() + 0.05
        #while pub.get_num_connections() == 0 and timeout_t > time.time():
        #    _sleep(0.01)
    
def _list_topics(print_out = True):
    def topic_type(t, topic_types):
        matches = [t_type for t_name, t_type in topic_types if t_name == t]
        if matches:
            return matches[0]
        return 'unknown type'
        
    master = rosgraph.Master('/rostopic')
    try:
        state = master.getSystemState()
        pubs, subs, _ = state
        #we want only the published topics
        topic_types = _master_get_topic_types(master)
        if print_out:
            print("\nPublished topics:")
            for t, l in pubs:
                if len(l) > 1:
                    print(" * %s [%s] %s publishers"%(t, topic_type(t, topic_types), len(l)))
                else:
                    print(" * %s [%s] 1 publisher"%(t, topic_type(t, topic_types)))    
    except socket.error:
        raise ROSTopicIOException("Unable to communicate with master!")
        
    return pubs

def _Resume_Page():
    #print pages_folder + "index.html"
    f = open(pages_folder + "index.html")
    str = f.read()
    #str.replace("IPTOBEEDITED", "")
    #print str
    f.close()
    return str
         

global g_list_topic_thrsh
g_list_topic_thrsh = 30
global g_list_topic_stamp 
g_list_topic_stamp = 0
# Handles the HTTP Requests for a list of topics
def _Ask_Topics(topics_list):
    global g_list_topic_stamp
    global g_list_topic_thrsh
    topics = topics_list.replace("%20", " ").split(" ")
    if not ROOT_NAMESPACE:
        for i in range(len(topics)):
            topics[i]=_namespace+topics[i]

    pubs=[]
    msg = ""
    list_once = False
    for i in topics[1:]:

        # get the information
        if ALL_TOPICS.count(i)>0:
            #print("Encontrou %s %s"%(i, msg))
            if not ROOT_NAMESPACE:
                name=i.replace(_namespace,'')
            else:
                name=i
            [topic_data, topic_time] = ALL_TOPICS.data_topic(i)
            if(topic_time != 0):
                msg += moos2xml(topic_data, name, topic_time, 0)
        tnow = rospy.Time.now()

        # if it is a new topic or g_list_topic_thrsh seconds passed
        if (SUBSCRIBED_TOPICS.count(i)==0 and ALL_TOPICS.count(i)==0 and UNKNOWN_TOPICS.count(i)==0 and not list_once) or \
            (((tnow-g_list_topic_stamp).to_sec() > g_list_topic_thrsh) and UNKNOWN_TOPICS.count(i)!=0):
            list_once = True
            pubs = _list_topics(False)
            g_list_topic_stamp = tnow

        if len(pubs)==0:
            continue
        for topic, l in pubs:
            if topic[1:]==i: # Verify if anyone is registered for publishing
                if (SUBSCRIBED_TOPICS.count(i)==0): # Register for the new topics
                    #print("Subscribing to topic %s"%topic)
                    SUBSCRIBED_TOPICS.add_value(topic, "")
                    subscriber_Thread(topic).start()
                    UNKNOWN_TOPICS.del_value(topic)
            elif(UNKNOWN_TOPICS.count(i)==0 and SUBSCRIBED_TOPICS.count(i)==0):
                #print ("UKN " + topic + " i " + i)
                UNKNOWN_TOPICS.add_value(i, "")
    
    # include the errors in the message
    msg +=ALL_ERRORs.__str__()
    
    #print msg;
    if msg == "":
        return "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<content>\n\n</content>\n\n"
    else:
        return "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<content>\n" + msg + "\n</content>\n\n"

def getfiles(dirpath):
    a = [s for s in os.listdir(dirpath)
         if os.path.isfile(os.path.join(dirpath, s))]
    return a

# Gerenerates an html code with all the missions in the mission FOLDER
def _List_Missions():
    # version 1 alphabetic sort
    missions = os.listdir(MISSION_PATH) # Directory Listing
    missions = sorted(missions, key=str.lower,reverse=True)
    # version 2 modified sort
    # missions = sorted(os.listdir(MISSION_PATH),key=lambda x: os.stat(os.path.join(MISSION_PATH,x)).st_mtime, reverse=True)
    # version 3 modified sort
    # missions=getfiles(MISSION_PATH)
    # missions = sorted(missions, key=lambda x: os.path.getmtime(os.path.join(MISSION_PATH,x)), reverse=True)
    res =   "<html>"+\
            "<head>"+\
            "   <meta charset=\"utf-8\">"+\
            "   <meta http-equiv=\"Content-Type\" content=\"text/html; charset=utf-8\">"+\
            "   <style>"+\
            "           * { margin: 0; padding: 0; }"+\
            "           html { height: 90%; }"+\
            "           body {font-size: 62.5%; font-family: Arial, Tahoma, sans-serif; color: #343434; padding-bottom: 0px; }"+\
            "           p { font-size: 1.3em; line-height: 1.5em; margin-bottom: 10px; }"+\
            "           a { text-decoration: none; }"+\
            "           img { border: 0; }"+\
            "           /** Navigation list **/"+\
            "           ul#nlists { list-style: none; font-family: \"Calibri\", Arial, sans-serif; }"+\
            "           ul#nlists li { "+\
            "               border-bottom: 1px solid #d9e2eb; "+\
            "               background: #fff; "+\
            "               display: block;"+\
            "               background-image: -webkit-gradient(linear, 0% 0%, 0% 100%, from(#ffffff) to(#ecf1f5));"+\
            "               background-image: -webkit-linear-gradient(top, #ffffff, #ecf1f5);"+\
            "               background-image:    -moz-linear-gradient(top, #ffffff, #ecf1f5);"+\
            "               background-image:      -o-linear-gradient(top, #ffffff, #ecf1f5);"+\
            "               background-image:         linear-gradient(top, #ffffff, #ecf1f5); "+\
            "           }"+\
            "           ul#nlists li a { "+\
            "               position: relative; "+\
            "               display: block;"+\
            "               box-sizing: border-box;"+\
            "               width: 100%;"+\
            "               padding: 15px 0px; "+\
            "               padding-left: 10px;"+\
            "               text-decoration: none; "+\
            "               font-size: 1.4em; "+\
            "               color: #787878;"+\
            "               font-weight: bold;"+\
            "               overflow: hidden;"+\
            "           }"+\
            "           ul#nlists li a:hover { color: #999; }"+\
            "           ul#nlists li a:hover::after { border-color: #c3ca80; }"+\
            "           #dropMission {"+\
            "               position: relative;"+\
            "               border: 1px solid black;"+\
            "               text-align: center;"+\
            "               vertical-align: middle;"+\
            "               display: inline-block;"+\
            "               width:69%;"+\
            "               height: 40px;"+\
            "               background: #DDDDDD;"+\
            "               -webkit-border-radius: 3px; -moz-border-radius: 3px; border-radius: 3px;"+\
            "           }"+\
            "           #dropMission input {"+\
            "               position: absolute;"+\
            "               vertical-align: middle;"+\
            "               width: 100%;"+\
            "               height: 100%;"+\
            "               top: 0;"+\
            "               left: 0;"+\
            "               opacity: 0;"+\
            "           }"+\
            "   </style>"+\
            "</head>"+\
            "<body>"+\
            "<form action=\"/\" enctype=\"multipart/form-data\" method=\"post\">"+\
            "   <input type=\"submit\" id=\"btUpload\" value='Upload' style=\"height: 40px; width: 20%; display: inline-block; \" >"+\
            "   <div id=\"dropMission\">"+\
            "       <p id=\"Mission_p\" style=\"font-weight: bold\">Drop Mission</p>"+\
            "       <input type=\"file\"  class=\"file\" id=\"MissionINP\" name=\"upfile\"/>"+\
            "   </div>"+\
            "</form>" +\
            "   <ul id=\"nlists\">"
    #print("MISSSIONS = %s"%missions)
    for f in missions:
        if f.endswith(".txt"):
            #print("Mission = [%s]"%(f));
            # Creating the list
            res += "<li><a href=\"/RSET Mission_Filename std_msgs/String "+MISSION_PATH+"/"+f+"\">"+f+"</a></li>\n"
    res += "   </ul>"+\
            "</body>"+\
            "</html>"
            
    return res
    
# One Thread for each subscription, it should take one of the mutexs
class subscriber_Thread( threading.Thread):
    def __init__(self, topic):
        self.topic = topic
        self.callback_echo = CallbackEcho(topic)
        self.msg_class, self.real_topic, self.msg_eval = get_topic_class(topic, blocking=True)
        if self.msg_class is None:
            # occurs on ctrl-C
            return
        threading.Thread.__init__(self)
            
    def run(self):      
        sub = rospy.Subscriber(self.real_topic, self.msg_class, self.callback_echo.callback, self.topic, queue_size=1)
        while not rospy.is_shutdown() and not self.callback_echo.done:
            _sleep(0.1)

def populenv(self):
        path = self.path
        dir, rest = '.', 'ciao'

        # find an explicit query string, if present.
        i = rest.rfind('?')
        if i >= 0:
            rest, query = rest[:i], rest[i+1:]
        else:
            query = ''

        # dissect the part after the directory name into a script name &
        # a possible additional path, to be stored in PATH_INFO.
        i = rest.find('/')
        if i >= 0:
            script, rest = rest[:i], rest[i:]
        else:
            script, rest = rest, ''

        # Reference: http://hoohoo.ncsa.uiuc.edu/cgi/env.html
        # XXX Much of the following could be prepared ahead of time!
        env = {}
        env['SERVER_SOFTWARE'] = self.version_string()
        env['SERVER_NAME'] = self.server.server_name
        env['GATEWAY_INTERFACE'] = 'CGI/1.1'
        env['SERVER_PROTOCOL'] = self.protocol_version
        env['SERVER_PORT'] = str(self.server.server_port)
        env['REQUEST_METHOD'] = self.command
        uqrest = urllib.unquote(rest)
        env['PATH_INFO'] = uqrest
        env['SCRIPT_NAME'] = 'ciao'
        if query:
            env['QUERY_STRING'] = query
        host = self.address_string()
        if host != self.client_address[0]:
            env['REMOTE_HOST'] = host
        env['REMOTE_ADDR'] = self.client_address[0]
        authorization = self.headers.getheader("authorization")
        if authorization:
            authorization = authorization.split()
            if len(authorization) == 2:
                import base64, binascii
                env['AUTH_TYPE'] = authorization[0]
                if authorization[0].lower() == "basic":
                    try:
                        authorization = base64.decodestring(authorization[1])
                    except binascii.Error:
                        pass
                    else:
                        authorization = authorization.split(':')
                        if len(authorization) == 2:
                            env['REMOTE_USER'] = authorization[0]
        # XXX REMOTE_IDENT
        if self.headers.typeheader is None:
            env['CONTENT_TYPE'] = self.headers.type
        else:
            env['CONTENT_TYPE'] = self.headers.typeheader
        length = self.headers.getheader('content-length')
        if length:
            env['CONTENT_LENGTH'] = length
        referer = self.headers.getheader('referer')
        if referer:
            env['HTTP_REFERER'] = referer
        accept = []
        for line in self.headers.getallmatchingheaders('accept'):
            if line[:1] in "\t\n\r ":
                accept.append(line.strip())
            else:
                accept = accept + line[7:].split(',')
        env['HTTP_ACCEPT'] = ','.join(accept)
        ua = self.headers.getheader('user-agent')
        if ua:
            env['HTTP_USER_AGENT'] = ua
        co = filter(None, self.headers.getheaders('cookie'))
        if co:
            env['HTTP_COOKIE'] = ', '.join(co)
        # XXX Other HTTP_* headers
        # Since we're setting the env in the parent, provide empty
        # values to override previously set values
        for k in ('QUERY_STRING', 'REMOTE_HOST', 'CONTENT_LENGTH',
                  'HTTP_USER_AGENT', 'HTTP_COOKIE', 'HTTP_REFERER'):
            env.setdefault(k, "")
        os.environ.update(env)


# Handles the HTTP Requests calling the specific functions
class HTTP_Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        try:
            #print("One Connection [GET %s]\n"% self.path)
            if self.path.strip()=="/" or self.path.startswith("/index"): 
                self.send_response(200)
                self.send_header('Content-type',    'text/html')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('cache-control', 'private, s-maxage=0, max-age=0, must-revalidate') #'cache-control': 'private, s-maxage=0, max-age=0, must-revalidate'
                self.end_headers()
                self.wfile.write(_Resume_Page().encode())

            elif self.path.endswith(".js"):
                f = open(pages_folder + self.path)  
                self.send_response(200)
                self.send_header('Content-type', 'application/javascript')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('cache-control', 'private, s-maxage=0, max-age=0, must-revalidate') #'cache-control': 'private, s-maxage=0, max-age=0, must-revalidate'
                self.end_headers()
                self.wfile.write(f.read().encode())
                f.close()

            elif self.path.endswith(".css"):
                f = open(pages_folder + self.path) 
                self.send_response(200)
                self.send_header('Content-type', 'text/css')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('cache-control', 'private, s-maxage=0, max-age=0, must-revalidate') #'cache-control': 'private, s-maxage=0, max-age=0, must-revalidate'
                self.end_headers()
                self.wfile.write(f.read().encode())
                f.close()
            
            elif self.path.startswith("/VAR" ): 
                self.send_response(200, "ok")
                self.send_header('Content-type',    'text/xml')
                self.send_header('Access-Control-Allow-Origin', '*') #("Cache-Control: no-cache, must-revalidate");
                self.send_header('Expires', 'Sat, 01 Jan 2005 00:00:00 GMT')
                self.send_header('Cache-Control', 'private, s-maxage=0, max-age=0, must-revalidate') #'cache-control': 'private, s-maxage=0, max-age=0, must-revalidate'
                self.end_headers()
                res = _Ask_Topics(self.path)
                self.wfile.write(res.encode())
            
            elif self.path.startswith("/RSET" ): # Set one variable as a rostopic pub format
                topic_send = cmd_set_topic(self.path)
                self.send_response(200)
                self.send_header('Content-type',    'text/html')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('cache-control', 'private, s-maxage=0, max-age=0, must-revalidate') #'cache-control': 'private, s-maxage=0, max-age=0, must-revalidate'
                self.end_headers()
                if topic_send: self.wfile.write('OK'.encode())
                else: self.wfile.write('NOK'.encode())

            elif (self.path.startswith("/action.html")):
                f = open(pages_folder + "action.html")
                query = urlparse(self.path).query
                query_components = dict(qc.split("=") for qc in query.split("&"))
                self.send_response(200)
                self.send_header('Content-type',    'text/html')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('action', query_components['action'])
                self.end_headers()
                self.wfile.write(f.read().encode())
                f.close()
                
            elif (self.path.endswith(".html") or self.path.endswith(".jpg") or self.path.endswith(".png") or self.path.endswith(".gif")): # Path Files
                #self.path has /test.html
                f = open(pages_folder+ "." + self.path)
                #print "Getting "+pages_folder+ "." + self.path
                #note that this potentially makes every file on your computer readable by the internet
                self.send_response(200)
                self.send_header('Content-type',    'text/html')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(f.read().encode())
                f.close()
                
            elif self.path.endswith("/Missions"): # List Missions
                res = _List_Missions()
                self.send_response(200)
                self.send_header('Content-type',    'text/html')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(res.encode())         
            else:
                print("File not found %s]\n"% self.path)
                self.send_error(404,'File Not Found: %s' % self.path)
            """if self.path.endswith(".esp"):   #our dynamic content
            self.send_response(200)
            self.send_header('Content-type',    'text/html')
            self.end_headers()
            self.wfile.write("hey, today is the" + str(time.localtime()[7]))
            self.wfile.write(" day in the year " + str(time.localtime()[0]))
            return
            """         
        except IOError:
            self.send_error(408,'Request timeout: %s' % self.path)

    def do_POST(self):
        try:
            populenv(self)
            form = cgi.FieldStorage(fp=self.rfile)
            upfilecontent = form['upfile'].value
            if upfilecontent:
                self.send_response(301)
                #print "filecontent:\n", upfilecontent
                print("[HTTP Server] Mission Uploaded " + MISSION_PATH+"/"+str(form['upfile'].filename))
                f = open(MISSION_PATH + "/" + str(form['upfile'].filename), "wb")
                self.end_headers()
                #fout = open(os.path.join('tmp', form['upfile'].filename), 'wb')
                f.write(upfilecontent)
                f.close()
                self.wfile.write("<HTML>Mission Uploaded! <br>".encode());
                _sleep(0.2)
                self.wfile.write(_List_Missions().encode());
                #self.wfile.write(upfilecontent[0]);
        except :
            self.wfile.write("<HTML>ERROR!</HTML>".encode());
            pass
    def do_HEAD(self):
        self.send_response(200)
        self.send_header('Content-type',    'text/html')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

    def address_string(self):
        host,port = self.client_address[:2]
        return host
    
    # Disabling Log Messages and screen Output
    def log_request(self, inputs):
        return ""

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""

# Dictionary to convert from signalnum to signal name (2 = SIGINT)
SIGNALS_TO_NAMES_DICT = dict((getattr(signal, n), n) \
    for n in dir(signal) if n.startswith('SIG') and '_' not in n )

# Function to handle signals
def signal_handler(signal, frame):
    print('[HTTP Server] Handling %s, closing HTTP_Server'%(str(SIGNALS_TO_NAMES_DICT[signal])))
    #sys.exit(0)
    os._exit(0)

if __name__ == '__main__':
    argv = sys.argv
    if argv is None:
        argv=sys.argv
    # filter out remapping arguments in case we are being invoked via roslaunch
    argv = rospy.myargv(argv)
    
    # process argv
    if len(argv) == 1:
        _fullusage()
    
    signal.signal(signal.SIGINT, signal_handler) # handle SIGINT

    _check_master()
    rospy.init_node(NAME, anonymous=True, disable_rosout=True, disable_rostime=True)
    # variable not to rostopic list for every request
    g_list_topic_stamp = rospy.Time.now()-rospy.Duration(g_list_topic_thrsh*2)
    MISSION_PATH = rospy.get_param('~Mission_Folder','/home/farol/farol-sw/Missions_FOLDER').replace('//','/') # solve the issue when the path is something like /home/cog/cog-sw//../Missions_FOLDER
    pages_folder = rospy.get_param('~pages_folder',"/home/farol/farol-sw/farol_common/http_server/pages/")
    server_port = rospy.get_param('~PORT',7080)

    ROOT_NAMESPACE = rospy.get_param('~ROOT_NAMESPACE',True)
    if not ROOT_NAMESPACE:
        _namespace=rospy.get_namespace()
        _namespace=_namespace[1:]
    else:
        _namespace=""
    
    _Ask_Topics("VAR rosout gps/data imu/data bat_monit/data ThrusterR/Status ThrusterL/Status Safety_Feature Leak1 Leak2 Pressure");
    try:
        command = argv[1]
        if command == 'echo':
            _rostopic_cmd_echo(argv)
        elif command in 'list':
            _list_topics()
        elif command in 'console':
            try:
                server = ThreadedHTTPServer(('', server_port), HTTP_Handler)
                sys.stdout.write("[HTTP Server] Try http://127.0.0.1:"+str(server_port)+"/\n")
                #server.serve_forever()
                while not rospy.is_shutdown():
                    server.handle_request();
            except KeyboardInterrupt:
                sys.stderr.write("[HTTP Server] KeyboardInterrupt\n")
                server.shutdown()
            #except socket.error, msg:
            except socket.error:
                sys.stderr.write("[HTTP Server] Socket Error: %s\n"% msg)
            except :
                sys.stderr.write("[HTTP Server] Unable to open the server\n")
                server.shutdown()
        else:
            _fullusage()
    except socket.error:
        sys.stderr.write("Network communication failed. Most likely failed to communicate with master.\n")
        sys.exit(1)
    except rosgraph.MasterException as e:
        # mainly for invalid master URI/rosgraph.masterapi
        sys.stderr.write("ERROR: %s\n"%str(e))
        server.shutdown()
        sys.exit(1)
    except ROSTopicException as e:
        sys.stderr.write("ERROR: %s\n"%str(e))
        server.shutdown()
        sys.exit(1)
    except KeyboardInterrupt: 
        server.shutdown()
    except rospy.ROSInterruptException: 
        server.shutdown()
