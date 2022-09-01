/******************************************************
 * CONSTANTS
 *****************************************************/

const READY_STATE = {REQ_NOT_INIT:0, CON_ESTABLISHED:1, REQ_RECEIVED:2, PROCESSING_REQ:3, REQ_FINISHED:4}
const REQ_STATUS = {OK:200, FORBIDDEN:403, NOT_FOUND:404}
const GREEN = "#00b33c"


/******************************************************
 * CLASSES
 *****************************************************/

// Topics list class
function topics_list()
{
	this.topics = {};
	this.addTopic = function(obj_topic)
	{
		this.topics[obj_topic.name] = obj_topic;
	}
}

// Topic class
function topic(name)
{
	this.name = name;
	this.subtopics = {};
	this.addSubTopic = function(name, fcn)
	{
		this.subtopics[name] = fcn;
	}
}


/******************************************************
 * GLOBAL VARIABLES
 *****************************************************/

// Topics name list
var varstoaks = ["ThrusterR/Status", "ThrusterL/Status", "can/motor_port", "can/motor_stbd", 
								 "can/battery_port", "can/battery_stbd", "gps/data", "imu/data", 
								 "leaks/port/1", "leaks/port/2", "leaks/stbd/1", "leaks/stbd/2"];

// Topics list object
var topics_list = new topics_list();

// Add port can topic to list
var can_port = new topic("can_motor_port");
can_port.addSubTopic('voltage',"pport_voltage");
topics_list.addTopic(can_port);

// Add stbd can topic to list
var can_stbd = new topic("can_motor_stbd");
can_stbd.addSubTopic('voltage',"pstbd_voltage");
topics_list.addTopic(can_stbd);

// Add left thruster topic to list
var thl = new topic("ThrusterL_Status");
thl.addSubTopic('Speed',"plm_speed");
thl.addSubTopic('Current',"plm_curr");
thl.addSubTopic('Errors',"plm_errors");
topics_list.addTopic(thl);

// Add right thruster topic to list
var thr = new topic("ThrusterR_Status");
thr.addSubTopic('Speed',"prm_speed");
thr.addSubTopic('Current',"prm_curr");
thr.addSubTopic('Errors',"prm_errors");
topics_list.addTopic(thr);

// Add port battery topic to list
var bat_port = new topic("can_battery_port");
bat_port.addSubTopic('current',"bp_curr");
bat_port.addSubTopic('voltage',"bp_volt");
bat_port.addSubTopic('min_voltage',"bp_min_volt");
bat_port.addSubTopic('max_voltage',"bp_max_volt");
bat_port.addSubTopic('temperature',"bp_temp");
topics_list.addTopic(bat_port);

// Add stbd battery topic to list
var bat_stbd = new topic("can_battery_stbd");
bat_stbd.addSubTopic('current',"bs_curr");
bat_stbd.addSubTopic('voltage',"bs_volt");
bat_stbd.addSubTopic('min_voltage',"bs_min_volt");
bat_stbd.addSubTopic('max_voltage',"bs_max_volt");
bat_stbd.addSubTopic('temperature',"bs_temp");
topics_list.addTopic(bat_stbd);

// Add IMU topic to list
var imu = new topic("imu_data");
imu.addSubTopic('Yaw',"pheading");
imu.addSubTopic('Gz',"pheading_rate");
topics_list.addTopic(imu);

// Add GPS topic to list
var gps = new topic("gps_data");
gps.addSubTopic('satellites',"psats");
gps.addSubTopic('latitude',"plat");
gps.addSubTopic('longitude',"plon");
topics_list.addTopic(gps);

// Add leaks port 1 topic to list
var leaks_port_1 = new topic("leaks_port_1");
leaks_port_1.addSubTopic('data',"lp1");
topics_list.addTopic(leaks_port_1);

// Add leaks port 2 topic to list
var leaks_port_2 = new topic("leaks_port_2");
leaks_port_2.addSubTopic('data',"lp2");
topics_list.addTopic(leaks_port_2);

// Add leaks stbd 1 topic to list
var leaks_stbd_1 = new topic("leaks_stbd_1");
leaks_stbd_1.addSubTopic('data',"ls1");
topics_list.addTopic(leaks_stbd_1);

// Add leaks stbd 2 topic to list
var leaks_stbd_2 = new topic("leaks_stbd_2");
leaks_stbd_2.addSubTopic('data',"ls2");
topics_list.addTopic(leaks_stbd_2);


/******************************************************
 * FUNCTIONS
 *****************************************************/

/**
 * Creates an XMLHttpRequest object to request data from a web server.
 *
 * @return {XMLHttpRequest} Object or null if browser does not allow (old).
 */
function getXMLHttpRequest()
{
	// Return the XMLHttpRequest by testing if the browser has the 
	// functionnality or not
	// If the object cannot be created, return null
	if (window.XMLHttpRequest) {
		return new window.XMLHttpRequest;
	} 
	else 
	{
		try 
		{
			return new ActiveXObject("MSXML2.XMLHTTP.3.0");
		}
		catch (ex) 
		{
			return null;
		}
	}
}

/**
 * Creates an HTTP request.
 *
 * @param 	{timer}						Timer object for the request timeouts.
 * @param 	{callback}				Function to be run when the request response was received.
 * @return 	{XMLHttpRequest}	Object or null if browser does not allow (old).
 */
function createHTTPreq(timer, callback){
	// Get XMLHttpRequest object to request data from web server
	var xmlhttp = getXMLHttpRequest();

	// Configure the XMLHttpRequest object
	if (xmlhttp){
		// Define the function to call when the request state changes
		xmlhttp.onreadystatechange = function() 
		{
			// If the request is not finished, do nothing
			if (xmlhttp.readyState != READY_STATE.REQ_FINISHED)
				return;

			// Otherwise clear the request timer timeout
			clearTimeout(timer);

			// And abort the request in case it is not OK
			if (this.status != REQ_STATUS.OK){
				this.abort();
				return;
			}
			
			// Read the request's XML content
			if (callback != null) callback(this);
		};

		return xmlhttp;
	}
	else return null;
}

/**
 * Displays the response to one of the action request (wash thrusters,
 * test thrusters, pc shutdown, etc).
 *
 * @param {XMLHttpRequest} xmlhttp Request object.
 */
function actionCallback(xmlhttp)
{
	// Read only if request is finished and its status is ok
	if(xmlhttp.readyState == READY_STATE.REQ_FINISHED && xmlhttp.status == REQ_STATUS.OK)
	{
		// Display response
		document.getElementById('req_status').innerHTML = xmlhttp.response;

		// Paint the status in red if it was not successfull
		if (xmlhttp.response == "OK") document.getElementById("req_status").style.color = GREEN;
	}
	xmlhttp.abort();
}


/**
 * Sends an action request to the server (pc shutdown, wash thrusters, etc)-
 */
function buttonAction()
{
	// Variables
	var requestTimer;
	var xmlhttp;

	// HTTP request
	xmlhttp = createHTTPreq(requestTimer, actionCallback);
	
	// Set the request object if it was correctly created
	if(xmlhttp)
	{
		// Set request parameters (type, url, asynchronous)
		urlParams = new URLSearchParams(window.location.search);
		action = urlParams.get('action');
		xmlhttp.open("GET", action, true);
		
		// Set request timeout
		requestTimer = setTimeout(function()
		{
			// After 800ms, if the request was successfull, read its content and then abort
			// Otherwise just abort
			if (xmlhttp.readyState == READY_STATE.REQ_FINISHED && xmlhttp.status == REQ_STATUS.OK) readXML(xmlhttp);
			xmlhttp.abort();
		}, 800);

		// Send request
		xmlhttp.send();
	}
}

/**
 * Open action page with reset request to test thrusters as argument.
 */
function testThrusters()
{
	window.location.href = "/action.html?action=./RSET Thruster_Test std_msgs/Bool true";
}

/**
 * Open action page with reset request to shut PC down as argument.
 */
function pcDown()
{
	// Open page to send request and display response
	window.location.href = "/action.html?action=./RSET bat_monit/pc_shutdown std_msgs/Bool true";
}

/**
 * Reads XML values received from web server after request and displays them.
 *
 * @param {?} topic 		Topic name.
 * @param {?} subtopic 	Subtopic name (variable of the topic's message)
 * @param {?} text 			Subtopic value.
 */
function updateFields(topic, subtopic, text)
{
	console.log("Topic: " + topic + ", subtopic:" + subtopic + ", data:" + text)
	
	if (topics_list.topics[topic] && topics_list.topics[topic].subtopics[subtopic])
	{
		// Extract value correctly depending on the topic and subtopic
		no_error = true;
		subtopic = new Array(subtopic);
		value = new Array();
		switch(topic+"_"+subtopic)
		{
			case "can_motor_port_voltage":
			case "can_motor_stbd_voltage":
			case "can_battery_port_current":
			case "can_battery_stbd_current":
			case "can_battery_port_temperature":
			case "can_battery_stbd_temperature":
			case "ThrusterL_Status_Speed":
			case "ThrusterR_Status_Speed":
			case "gps_data_satellites":
				value[0] = parseInt(text);
				break;
			case "can_battery_port_voltage":
			case "can_battery_stbd_voltage":
				// Correct subtopic
				subtopic = new Array("min_voltage", "max_voltage");

				// Treat values
				text = text.replace("(", "");
				text = text.replace(")", "");
				volt = text.split(",")

				// Find min and max
				min = max = parseInt(volt[0]);
				for (i = 1; i < volt.length; i++)
				{
					val = parseInt(volt[i]);
					if (val > max) max = val;
					if (val < min) min = val;
				}

				value = [min, max];
				break;
			case "gps_data_longitude":
			case "gps_data_latitude":
				value[0] = parseFloat(text).toFixed(6)+'';
				break;
			case "ThrusterL_Status_Errors":
			case "ThrusterR_Status_Errors":
				error = parseInt(text);
				if (error == 0) value[0] = "none";
				else
				{ 
					no_error = false;
					value[0] = "";
					if (error & 0b0100) value[0] = value[0] + "hall "; 
					if (error & 0b1000) value[0] = value[0] + "leak ";
					if (value[0] == "") value[0] = "unknown";
				}
				break;
			case "leaks_port_1_data":
			case "leaks_port_2_data":
			case "leaks_stbd_1_data":
			case "leaks_stbd_2_data":
				if (text == "False")
				{
					value[0] = "no leak";
				}
				else if (text == "True")
				{
					value[0] = "LEAK !";
					no_error = false;
				}
				break;
			default:
				value[0] = parseFloat(text).toFixed(2)+'';
		}

		// Display values
		for (i = 0; i < value.length; i++)
		{
			document.getElementById(topics_list.topics[topic].subtopics[subtopic[i]]).innerHTML = value[i];
			if(no_error) document.getElementById(topics_list.topics[topic].subtopics[subtopic[i]]).style.color = GREEN;
		}
	}
}

/**
 * Display NA in all topic cells (clear all values).
 */
function clearFields()
{
	for (var topic in topics_list.topics) 
	{
		for (var subtopic in topics_list.topics[topic].subtopics)
		{
			if (document.getElementById(topics_list.topics[topic].subtopics[subtopic]))
				document.getElementById(topics_list.topics[topic].subtopics[subtopic]).innerHTML = "NA";
		}
	}
}

/**
 * Reads XML received from web server after topic values request.
 *
 * @param {XMLHttpRequest} xmlhttp Request object.
 */
function askVarsCallback(xmlhttp)
{
	
	console.log("Reading xml");
	// Read only if request is finished and its status is ok
	if(xmlhttp.readyState == READY_STATE.REQ_FINISHED && xmlhttp.status == REQ_STATUS.OK)
	{
		// Clear all fields of the table
		clearFields();

		// Get the XML content
		xmlDoc = xmlhttp.responseXML;

		// Start reading the XML content if it is not empty
		if(xmlDoc)
		{
			// Check if XML contains a ROSMessage tags
			if(xmlDoc.getElementsByTagName("ROSMessage").length != 0)
			{
				// Get the ROS message list of topics
				var rosmsg = xmlDoc.getElementsByTagName("ROSMessage");

				// Go through each topic
				for(var i = 0; i < rosmsg.length; i++)
				{
					// Get topic name
					name_nodeValue = rosmsg[i].attributes.getNamedItem("name").value;

					// Get list of variables in the topic message (subtopics)
					var var_elements = rosmsg[i].getElementsByTagName("KEY");

					// Go through all the variables of the message (subtopic)
					for(var j = 0; j < var_elements.length; j++)
					{
						// Helping vars
						var subtopic = "";
						var topicvalue = "";

						// Get the value of the subtopic
						// Second case is IE only, first is for all the other browsers
						if(var_elements[j].textContent)
						{
							subtopic = var_elements[j].textContent;
							topicvalue = rosmsg[i].getElementsByTagName("DOUBLE")[j].textContent;
						}
						else
						{
							subtopic = var_elements[j].text;
							topicvalue = rosmsg[i].getElementsByTagName("DOUBLE")[j].text;
						}

						// Update value if valid
						updateFields(name_nodeValue, subtopic, topicvalue);
					}
				}
			}
		}
	}
	xmlhttp.abort();
}

/**
 * Sends request to the server to get the ROS topics values.
 * @todo Should not create the request object each time, but only resend it.
 */
function askVars()
{
	// Variables
	var requestTimer;
	var xmlhttp;

	// HTTP request
	xmlhttp = createHTTPreq(requestTimer, askVarsCallback);
	
	// Set the request object if it was correctly created
	if(xmlhttp)
	{
		// Set request parameters (type, url, asynchronous)
		// In our case our URL is "/VAR20%topic_name_1%20topic_name_2.....20%"
		xmlhttp.open("GET", "/VAR%20" + varstoaks.join("%20"), true);
		
		// Set request header
		xmlhttp.setRequestHeader("If-Modified-Since", "Sat, 1 Jan 2005 00:00:00 GMT");
		
		// Set request timeout
		requestTimer = setTimeout(function()
		{
			// After 800ms, if the request was successfull, read its content and then abort
			// Otherwise just abort
			if (xmlhttp.readyState == READY_STATE.REQ_FINISHED && xmlhttp.status == REQ_STATUS.OK) askVarsCallback(xmlhttp);
			xmlhttp.abort();
		}, 800);

		// Send request
		xmlhttp.send();
	}
}

/**
 * Initializes the javascript code by calling the askVars() function
 * every 1s (1000ms).
 */
function init()
{
	setInterval("askVars();",1000);
}