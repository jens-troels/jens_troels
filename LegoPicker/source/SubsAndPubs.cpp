//
// Created by jeh on 9/30/19.
//

#include "SubsAndPubs.h"


SubsAndPubs::SubsAndPubs(int argc, char **argv)
{
    ros::init(argc, argv, "RSD2019");
    ros::NodeHandle _n;

    signalRoboPub = _n.advertise<std_msgs::Int32>(signalRoboTopic, 1);
    idPub = _n.advertise<std_msgs::Int32>(idTopic, 1);
    ticketPub = _n.advertise<std_msgs::String>(ticketTopic, 1);
    colorPub = _n.advertise<std_msgs::Int32>(PIColorTopic, 1);
    lightPub = _n.advertise<std_msgs::Int32>(lightTopic, 1);
    gripperPub = _n.advertise<std_msgs::Bool>(gripperTopic, 1);
    roboMirPub = _n.advertise<std_msgs::Int32>(roboMirTopic, 1);
    packMLPub = _n.advertise<std_msgs::Int32>(packMLTopic, 1);
    guiPub = _n.advertise<std_msgs::Int32>(guiTopic, 1);
    fakeMirPub = _n.advertise<std_msgs::Int32>(mirRoboTopic,1);
    


    signalMESSub = _n.subscribe(signalMESTopic, 1, &SubsAndPubs::callbackSig, this);
    idSub = _n.subscribe(idTopic, 1, &SubsAndPubs::callbackId, this);
    ticketSub = _n.subscribe(ticketTopic, 1, &SubsAndPubs::callbackTicket, this);
    PISignalSub = _n.subscribe(PISignalTopic, 1, &SubsAndPubs::callbackPISig, this);
    brickSub = _n.subscribe(brickTopic, 1, &SubsAndPubs::callbackBricks, this);
    mirRoboSub = _n.subscribe(mirRoboTopic, 1, &SubsAndPubs::callbackMir, this);
    packMLSub = _n.subscribe(packMLTopic, 1, &SubsAndPubs::callbackPackML, this);
    guiSub = _n.subscribe(guiTopic, 1, &SubsAndPubs::callbackGUI, this);
    emSub = _n.subscribe(emTopic, 1, &SubsAndPubs::callbackEm, this);
    ackSub = _n.subscribe(ackTopic, 1, &SubsAndPubs::callbackAck, this);



    ros::Rate loop_rate(10); //Arbitrary rate, maybe fine tuning needed, idk

}

void SubsAndPubs::callbackPackML(const std_msgs::Int32 &_state) {
    state = _state.data;
    waitGugi(0.2, TOGGLE_HOLD);

}

void SubsAndPubs::callbackAck(const std_msgs::Bool _ack) {
    ackFlag = _ack.data;
}

void SubsAndPubs::callbackEm(const std_msgs::Int32 &_emSig)
{
    emSig = _emSig.data;
    if(emSig == 0)
        setState(ABORTING);
}

int SubsAndPubs::getEmSig()
{
    return emSig; 
}

bool SubsAndPubs::getAck()
{
    return ackFlag;
}

void SubsAndPubs::topicSleep(double _sleepy)
{
    double tickCounter = 0;
    while(tickCounter < _sleepy)
        tickCounter++;
}


void SubsAndPubs::callbackMir(const std_msgs::Int32::ConstPtr &_mirSig)
{
    mirSig = _mirSig->data; 
}

int SubsAndPubs::getMirSig()
{
    return mirSig;
}

void SubsAndPubs::callbackPISig(const std_msgs::String::ConstPtr& _PIsignal){
    PISignal = _PIsignal->data;
}

void SubsAndPubs::callbackSig(const std_msgs::Int32::ConstPtr &_signal)
{
    MESSignal = _signal->data;
    waitFlag = true;
}

void SubsAndPubs::callbackGUI(const std_msgs::Int32::ConstPtr &_guiSig) {
    guiSig = _guiSig->data;
    if(guiSig == 1)
    {
        setState(HOLDING);
    }
    if(guiSig == 2)
    {
        setState(RESETTING);
    }
}

void SubsAndPubs::callbackId(const std_msgs::Int32& _id)
{
    currentId = _id.data;
}

void SubsAndPubs::callbackTicket(const std_msgs::String::ConstPtr& _ticket)
{
    currentTicket = _ticket->data;
}

void SubsAndPubs::callbackBricks(const std_msgs::String::ConstPtr& _bricks)
{
    brickVector = _bricks->data;
}


void SubsAndPubs::rosSpinOnce()
{
    ros::spinOnce();
}

int SubsAndPubs::getCurrentId() {

    return currentId;
}

std::string SubsAndPubs::getCurrentTicket() {
    return currentTicket;
}

int SubsAndPubs::getMESSignal() {
    return MESSignal;
}


std::string SubsAndPubs::getPISignal() {
    return PISignal;
}

std::string SubsAndPubs::getBrickVector() {
    return brickVector;
}

int SubsAndPubs::getGUISig() {
    return guiSig;
}

void SubsAndPubs::setFakeMir(int _fakeMirSig)
{
    std_msgs::Int32 fakeMirSig;
    fakeMirSig.data = _fakeMirSig;
    fakeMirPub.publish(fakeMirSig);
    ros::spinOnce();
}

void SubsAndPubs::setMirSignal(int _mirSignal)
{
    std_msgs::Int32 mirSignal;
    mirSignal.data = _mirSignal;
    roboMirPub.publish(mirSignal);
    ros::spinOnce();
}

int SubsAndPubs::getState() {
    return state;
}

void SubsAndPubs::setColor(int _color) {
    std_msgs::Int32 color;
    color.data = _color;
    colorPub.publish(color);
    usleep(SECONDS * 0.3);; //TODO usleep is in microseconds, maybe 0.5 seconds is enough?
    ros::spinOnce();
}

void SubsAndPubs::setId(int _id) {
    std_msgs::Int32 id;
    id.data = _id;
    idPub.publish(id);
    rosSpinOnce();
}

void SubsAndPubs::setMESSignal(int _signal) {
    std_msgs::Int32 signal;
    signal.data = _signal;
    signalRoboPub.publish(signal);
    rosSpinOnce();
}

void SubsAndPubs::setTicket(std::string _ticket) {
    std_msgs::String ticket;
    ticket.data = _ticket;
    ticketPub.publish(ticket);
    rosSpinOnce();
}

void SubsAndPubs::setLight(int _light){
    std_msgs::Int32 light;
    light.data = _light;
    lightPub.publish(light);
    rosSpinOnce();
}

void SubsAndPubs::setGripper(bool _flag){
    std_msgs::Bool flag;
    flag.data = _flag;
    gripperPub.publish(flag);
    rosSpinOnce();
}

void SubsAndPubs::setState(int _state) {
    std_msgs::Int32 state;
    state.data = _state;
    packMLPub.publish(state);
    usleep(SECONDS * 0.75);
    rosSpinOnce();
}

void SubsAndPubs::setGUI(int _guiSig)
{
    std_msgs::Int32 gui;
    gui.data = _guiSig;
    guiPub.publish(gui);
    usleep(SECONDS * 0.75);
    rosSpinOnce();
}

void SubsAndPubs::waitMir(double sec, int mirSignal_)
{
    clock_t endwait;
    endwait = clock() + sec * CLOCKS_PER_SEC;
    while (clock() < endwait) {}
    rosSpinOnce();
    int _mirSignal = getMirSig();
    if(_mirSignal != mirSignal_)
        waitMir(sec, mirSignal_);
}

void SubsAndPubs::waitGugi(double sec, int gugiSig)
{
    clock_t endwait;
    endwait = clock() + sec * CLOCKS_PER_SEC;
    while (clock() < endwait) {}
    rosSpinOnce();
    int gugi = getGUISig();
    if(gugi == gugiSig)
        waitGugi(sec, gugiSig);
}

void SubsAndPubs::waitStart(double sec, int gugiSig)
{
    clock_t endwait;
    endwait = clock() + sec * CLOCKS_PER_SEC;
    while (clock() < endwait) {}
    rosSpinOnce();
    int gugi = getGUISig();
    if(gugi != gugiSig)
        waitStart(sec, gugiSig);
}

void SubsAndPubs::waitAck(double sec, bool gripperFlag) {
    setGripper(gripperFlag);
    clock_t endwait;
    endwait = clock() + sec * CLOCKS_PER_SEC;
    while (clock() < endwait) {}
    rosSpinOnce();
    bool _ack = getAck();
    if(!_ack)
        waitAck(sec, gripperFlag);

}

SubsAndPubs::~SubsAndPubs()
{
    ros::shutdown();
}