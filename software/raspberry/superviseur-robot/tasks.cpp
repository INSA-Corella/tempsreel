/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
// Valeur basse => plus grande priorité
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 10
#define PRIORITY_TBAT 15 
#define PRIORITY_TCAM 20
#define PRIORITY_TCAMIMG 20

#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCALCPOS 20 //a changer
#define PRIORITY_TSTOPCALC 20 //a changer

/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

int cptErr=0;
bool arenaSaved=false;
/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    //init cam
    camera= new Camera(sm,5);
    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_cpterr, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_cam, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_bat, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openBat, "th_openBat", 0, PRIORITY_TBAT, 0)) {//petite prio car pas critique
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openCam, "th_openCam", 0, PRIORITY_TCAM, 0)) {//petite prio car pas critique
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_closeCam, "th_closeCam", 0, PRIORITY_TCAM, 0)) {//petite prio car pas critique
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_getImgCam, "th_getImgCam", 0, PRIORITY_TCAMIMG, 0)) {//petite prio car pas critique
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_calcPos, "th_calcPos", 0, PRIORITY_TCALCPOS, 0)) {//petite prio car pas critique
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openBat, (void(*)(void*)) & Tasks::OpenBat, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    /***if (err = rt_task_start(&th_openCam, (void(*)(void*)) & Tasks::OpenCam, this)) {
    *    cerr << "Error task start: " << strerror(-err) << endl << flush;
    *    exit(EXIT_FAILURE);
    *}
    *if (err = rt_task_start(&th_closeCam, (void(*)(void*)) & Tasks::CloseCam, this)) {
    *    cerr << "Error task start: " << strerror(-err) << endl << flush;
    *    exit(EXIT_FAILURE);
    }***/
    if (err = rt_task_start(&th_getImgCam, (void(*)(void*)) & Tasks::getImgCam, this)) {
            cerr << "Error task start: " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
        }
    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
    CloseCam();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}
bool findingArena=false;
bool calculatePos=false;
bool batSend=false;

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            delete(msgRcv);
            exit(-1);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        }else if(msgRcv->CompareID(MESSAGE_CAM_OPEN)){
            OpenCam();
        }else if(msgRcv->CompareID(MESSAGE_CAM_CLOSE)){
            CloseCam();
        }else if(msgRcv->CompareID(MESSAGE_ROBOT_BATTERY_GET)){
            rt_mutex_acquire(&mutex_bat, TM_INFINITE);
            batSend=true;
            rt_mutex_release(&mutex_bat);
            cout << "on a lance la fonction!" << endl << flush; 
        }else if(msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)){
            findingArena=true;
        }else if(msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)){
            rt_mutex_acquire(&mutex_arenaSaved, TM_INFINITE);
            arenaSaved=true;
            rt_mutex_release(&mutex_arenaSaved);
            
        }else if(msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)){
            rt_mutex_acquire(&mutex_arenaSaved, TM_INFINITE);
            arenaSaved=false;
            rt_mutex_release(&mutex_arenaSaved);
        }else if(msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)){
            
            rt_mutex_acquire(&mutex_calculatePos, TM_INFINITE);
            calculatePos=true;
            rt_mutex_release(&mutex_calculatePos);
        }else if(msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)){
            
            rt_mutex_acquire(&mutex_calculatePos, TM_INFINITE);
            calculatePos=false;
            rt_mutex_release(&mutex_calculatePos);
        }else if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            cout << "Communication with monitor lost!" << endl << flush;
            close_communication_robot();      
            
        }
        //cout << "msg ! "<< (*msgRcv).ToString() << endl << flush;
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;
        rt_mutex_acquire(&mutex_cpterr, TM_INFINITE);
        cptErr=0;
        rt_mutex_release(&mutex_cpterr);
        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK); //msg superviseur --> moniteur
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        cout << "Start robot without watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithoutWD());
        testError((*msgSend).GetID());
        rt_mutex_release(&mutex_robot);
        
        cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }
    }
}
void Tasks::testError(int msgID){
    if (msgID == MESSAGE_ANSWER_ROBOT_TIMEOUT or msgID==MESSAGE_ANSWER_ROBOT_UNKNOWN_COMMAND or msgID==MESSAGE_ANSWER_ROBOT_ERROR or msgID==MESSAGE_ANSWER_COM_ERROR){
            rt_mutex_acquire(&mutex_cpterr, TM_INFINITE);
            cptErr++;
            cout << endl << "on augmente counter " << endl << flush;
            if (cptErr >=3){
                close_communication_robot();
                
            }
            rt_mutex_release(&mutex_cpterr);
        }else{
            rt_mutex_acquire(&mutex_cpterr, TM_INFINITE);
            cptErr=0;
            rt_mutex_release(&mutex_cpterr);
        }
}


void Tasks::close_communication_robot(){
    robot.Close(); 
    rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
    robotStarted=0;
    rt_mutex_release(&mutex_robotStarted);
   
    cout << "CO avec robot perdue :c " << endl << flush;            
}
/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            Message * msgSend;
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(new Message((MessageID)cpMove));
            testError((*msgSend).GetID());
            rt_mutex_release(&mutex_robot);
            
        }
        cout << endl << flush;
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}



void Tasks::OpenBat(){
    //declaration
    int bat;
    int rs;
    Message * batMsg;
    
    
    rt_sem_p(&sem_barrier, TM_INFINITE);//sem pour revenir au run
    //init la periode 
    rt_task_set_periodic(NULL, TM_NOW, 500000000);
    //faire while 1 avec waitPeriod
    while(1){
        //attend la periode donné 2 ligne plus haut
        rt_task_wait_period(NULL);
        bool traitement=false;
        rt_mutex_acquire(&mutex_bat, TM_INFINITE);
        traitement=batSend;
        rt_mutex_release(&mutex_bat);
        
        if(traitement){
        //verify that the connection to the robot started
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            rs = robotStarted;
            rt_mutex_release(&mutex_robotStarted);
        
            if (rs == 1) {

                //get battery level

                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                batMsg=robot.Write(new Message((MessageID)MESSAGE_ROBOT_BATTERY_GET));
                testError((*batMsg).GetID());
                rt_mutex_release(&mutex_robot);

                //montre le cout : debug
                //cout << " batterie: " << (*batMsg).ToString() << endl << flush;

                //send the bat
                WriteInQueue(&q_messageToMon, batMsg);
            }
        }
        
    }
}
    
    
    
bool camOpen=false;
    
void Tasks::OpenCam(){
    
    //declaration
    Message * camMsg;
    int err;
    
    
    //open camera 
    
    
    //open cam
    rt_mutex_acquire(&mutex_cam, TM_INFINITE);
    (*camera).Open();
    rt_mutex_release(&mutex_cam);
    //debug
    cout << " CAM OPEN ? : " << (*camera).IsOpen() << endl << flush;
    if((*camera).IsOpen()){
        camOpen=true;
        
    }else{
        cout << " CAM NOT OPEN : " << endl << flush;
        camOpen=false;
    }
    
    
    
}

void Tasks::CloseCam(){
    //declaration
    Message * camMsg;
    
    
    
    //close cam
    rt_mutex_acquire(&mutex_cam, TM_INFINITE);
    (*camera).Close();
    rt_mutex_release(&mutex_cam);
    //debug
    cout << " CAM open ? : " << (*camera).IsOpen() << endl << flush;

}

Arena arenaSave;

void Tasks::getImgCam(){
    //declaration
    
    //rt_sem_p(&sem_barrier, TM_INFINITE);
    
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while(1){
        //attend la periode donné 2 ligne plus haut
        rt_task_wait_period(NULL);
        
        //get image cam
        if(camOpen){
            rt_mutex_acquire(&mutex_cam, TM_INFINITE);
            Img img=(*camera).Grab();
            rt_mutex_release(&mutex_cam);
            
            //arena
            rt_mutex_acquire(&mutex_arenaSaved, TM_INFINITE);
            if(!arenaSaved){//recherche arene
                Arena arena= img.SearchArena();
                if(arena.IsEmpty()){//arena pas trouvée
                    Message * msg=new Message((MessageID)MESSAGE_ANSWER_NACK);

                    cout << " pas d'arene trouvée: " << endl << flush;
                    WriteInQueue(&q_messageToMon, msg);
                }else{
                    img.DrawArena(arena);
                    arenaSave=arena;
                    cout << " ############################ arene trouvée et dessinée ############################ ############################ : " << endl << flush;

                    Message * msg=new Message((MessageID)MESSAGE_ANSWER_ACK);

                    cout << " pas d'arene trouvée: " << endl << flush;
                }
            }else{
                 img.DrawArena(arenaSave);

                 cout << " arene dessinée: " << endl << flush;
                
            }
            rt_mutex_release(&mutex_arenaSaved);
            
            //Position
            rt_mutex_acquire(&mutex_calculatePos, TM_INFINITE);
            if(calculatePos and arenaSaved){//recherche position
                CalcPos(&img);
            }
            rt_mutex_release(&mutex_calculatePos);
            

            //create msg 

            MessageImg * msgImg=new MessageImg(MESSAGE_CAM_IMAGE,&img);

            
            (*msgImg).SetImage(&img);
            //send img to monitor
            WriteInQueue(&q_messageToMon, msgImg);

            //debug
            cout << " image sent: " << endl << flush;
        }
    }
}
    

    
    void Tasks::CalcPos(Img * img){
        //debug
        cout << " calcul de la pos sent: " << endl << flush;
        
        list<Position> listPos= (*img).SearchRobot(arenaSave);
        if(!listPos.empty()){//arena pas trouvée
            cout << listPos.size()<<" robot à dessiner: " << endl << flush;
            
            for(Position p: listPos){
                int id=p.robotId;
                bool robTrouver=false;
                if(id==11){//id du bon robot
                    //create msg 
                    MessagePosition * msgPos=new MessagePosition(MESSAGE_CAM_POSITION,p);
                   
                    
                    cout << " pos du robot envoyé: " << endl << flush;
                    //send msgPos to monitor
                    WriteInQueue(&q_messageToMon, msgPos);
                    robTrouver=true;
                }
                
            }
            //if(!robTrouver){
                
            //}
            int nbRobotDrawn =(*img).DrawAllRobots(listPos);
            cout << nbRobotDrawn <<" robot dessiné(s): " << endl << flush;
                //cout << listPos.size()<<" robot à dessiner: " << endl << flush;
                
                //int nbRobotDrawn =(*img).DrawAllRobots()
                //cout << nbRobotDrawn <<" robot dessiné(s): " << endl << flush;
                
        }
        
    }
    


