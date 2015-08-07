from __future__ import division
import serial, time
import socket
import csv
import Throughput
import os.path
from difflib import Differ
import sys
import os
import re

#initialization and open the port
#possible timeout values:
#    1. None: wait forever, block call
#    2. 0: non-blocking mode, return immediately
#    3. x, x is bigger than 0, float allowed, timeout block call

#====serial settings
import telnetlib
import TelnetController
import time
import re
import csv
import numpy as np

class Connect:


    def __init__(self):
        self.TestId=''
        self.TestStatus=''
        self.sport=''
        self.comport=''
        self.interface=''
        self.srate=''
        self.sbytesize=''
        self.sparity=''
        self.sstopbits=''
        self.sxonxoff=''
        self.srtscts=''
        self.sdsrdtr=''
        self.host=''
        self.tcport=''
        self.packlength=''
        self.iterations=''
        self.filesend=''
        self.buffer=1024
        
        iplist = open('properties')
        for ip in iplist.readlines():
            self.telnet = TelnetController.TelnetController(host_name = ip.strip(), user_name = 'admin', password = 'admin', prompt = '#')
            self.telnet.login()

        
    def stringTObool(self,val):
        if val=='False':
            return False
        else:
            return True
        
    def timeTOwait(self,baudrate,field):
        bits=len(field)*10
        wait=6
        if baudrate<=bits:
            wait=bits/baudrate+15
        return wait
        
    
    def serial_interpreter(self,flag,ser):
        if flag==1:
            if self.sbytesize=='8':
                ser.bytesize=serial.EIGHTBITS
            elif self.sbytesize=='7':
                ser.bytesize=serial.SEVENBITS
            elif self.sbytesize=='6':
               ser.bytesize=serial.SIXBITS
            elif self.sbytesize=='5':
               ser.bytesize=serial.FIVEBITS
            if self.sparity=='none':
               ser.parity=serial.PARITY_NONE
            if self.sparity=='odd':
               ser.parity=serial.PARITY_ODD
            if self.sparity=='even':
               ser.parity=serial.PARITY_EVEN
               #ser.parity=self.sparity.lower()
            if self.sstopbits=='1':
               ser.stopbits=serial.STOPBITS_ONE
            if self.sstopbits=='2':
               ser.stopbits=serial.STOPBITS_TWO
        elif flag==0:
            if self.sbytesize==serial.EIGHTBITS:
                self.sbytesize='8'
            elif self.sbytesize==serial.SEVENBITS:
                self.sbytesize='7'
            elif self.sbytesize==serial.SIXBITS:
                self.sbytesize='6'
            elif self.sbytesize==serial.FIVEBITS:
                self.sbytesize='5'
            if self.sparity==serial.PARITY_NONE:
               self.sparity='none'
            if self.sparity==serial.PARITY_ODD:
               self.sparity='odd'
            if self.sparity==serial.PARITY_EVEN:
               self.sparity='even'
            if self.sstopbits==serial.STOPBITS_ONE:
               self.sstopbits='1'
            if self.sstopbits==serial.STOPBITS_TWO:
               self.sstopbits='2'


            
    def setserial(self,sport,srate,sxonxoff,srtscts,sdsrdtr):
        ser = serial.Serial()
        ser.port = sport
        ser.baudrate = srate
        
        self.serial_interpreter(1,ser)
        #ser.timeout = None          #block read
        ser.timeout = 0              #non-block read
        #ser.timeout = 2             #timeout block read
        ser.xonxoff =sxonxoff     #disable software flow control
        ser.rtscts = srtscts     #disable hardware (RTS/CTS) flow control
        ser.dsrdtr = sdsrdtr       #disable hardware (DSR/DTR) flow control
        ser.writeTimeout = 0     #timeout for write
        return ser

    def settcp(self,host,tcport,buffer):
        TCP_IP =host 
        TCP_PORT = tcport
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((host, tcport))
        return s


    def telnetsession(self,comport,srate,sbytesize,interface,sparity,sstopbits,tcpport):
        
        vgateway=re.findall('([0-9]+)', comport)
        self.telnet.run_command('set serial '+comport+' '+'baudrate '+srate,0)
        self.telnet.run_command('set serial '+comport+' '+'databits '+sbytesize,0)
        self.telnet.run_command('set serial '+comport+' '+'interface '+interface,0)
        self.telnet.run_command('set serial '+comport+' '+'parity '+sparity,0)
        self.telnet.run_command('set serial '+comport+' '+'stopbits '+sstopbits,0)
        self.telnet.run_command('set vgateway '+''.join(vgateway)+' '+'rawtcpserver localport'+' '+tcpport,0)
        self.telnet.run_command('apply config',0)
        #telnet.run_command('exit',0)
        

    def logger(self,file,stream):
        file = open('Logger.txt', "a")
        file.write(stream)
        return file
        
        
    def sendreceive(self,interface):

        count_pckt=0
        count_pct_rcv=0
        count_pct_rcv1=0
        iteration=0
        list_time=list()
        list_time1=list()
        d=Differ()
        self.logger('Logger.txt', '  *'*10+'  Test Name: '+self.TestId+'  *'*10+'\n')

        self.telnetsession(self.comport, self.srate, self.sbytesize, self.interface, self.sparity, self.sstopbits,self.tcport)
        s=self.settcp(self.host,int(self.tcport),self.buffer)

        ser=self.setserial(self.sport,int(self.srate),self.stringTObool(self.sxonxoff.lower().capitalize()),self.stringTObool(self.srtscts.lower().capitalize()),self.stringTObool(self.sdsrdtr.lower().capitalize()))
        self.serial_interpreter(0,ser)
        self.logger('Logger.txt', '='*200+'\n'+'\n')
        self.logger('Logger.txt', 'Client COM Port: '+self.sport+'\n')
        self.logger('Logger.txt', 'Device COM Port: '+self.comport+'\n')
        self.logger('Logger.txt', 'Interface: '+self.interface+'\n')
        self.logger('Logger.txt', 'Baudrate: '+self.srate+'\n')
        self.logger('Logger.txt', 'Device IP: '+self.host+'\n')
        self.logger('Logger.txt', 'Device TCP port: '+self.tcport+'\n')
        self.logger('Logger.txt','='*200+'\n')

        #if interface=='rs232' or 'rs422' or 'rs485-2w' or 'rs485-4w' :
    
        try: 
            ser.open()
        except Exception, e:
            print sys.exc_traceback.tb_lineno      
            print "error open serial port: " + str(e)
            exit()
        if ser.isOpen():
            try:
                ser.flushInput() #flush input buffer, discarding all its contents
                ser.flushOutput()#flush output buffer, aborting current output    #and discard all that is in buffer

                while iteration<int(self.iterations):
                    iteration=iteration+1 ## contor send packets
                    filesend= open('filesend.txt')
                    self.logger('Logger.txt','='*200+'\n')
                    self.logger('Logger.txt','Iterations: '+str(iteration)+'\n')
                    self.logger('Logger.txt','='*200+'\n')
                    if interface=='loopback':
                        self.logger('Logger.txt','   Write data from serial to tcp   '+'\t'*20+'   Write data from tcp to tcp in loop:   '+'\n')
                    else: self.logger('Logger.txt','   Write data from serial to tcp   '+'\t'*20+'   Write data from tcp to serial:   '+'\n')
                    #self.logger('Logger.txt','='*200+'\n')
                    count_pckt=0
                    count_pct_rcv=0
                    count_pct_rcv1=0
                    for field in filesend.readlines():
                        count_pckt=count_pckt+1
                        send_data=''.join(field).replace('\n','')
                        ser.write(send_data)
                        time_send=time.time()
                        #print("Time send serial to tcp"+str(time_send)+" Write data: "+send_data)
                        #print '\n'
                        ser.flushInput() #flush input buffer, discarding all its contents
                        ser.flushOutput()#flush output buffer, aborting current output 
                        time.sleep(self.timeTOwait(int(self.srate),send_data))
                        data = s.recv(self.buffer)
                        s.close()
                        s=self.settcp(self.host,int(self.tcport),self.buffer)
                        data=data.replace('\n','')
                        time_received=time.time()
                        s.send(send_data.replace('\n', ''))
                        time_send1=time.time()
                        
                        
                        #print("Time send serial to tcp"+str(time_send)+" Write data: "+send_data)
                        #print '\n'
                        time.sleep(self.timeTOwait(int(self.srate),send_data))
                        if interface=='loopback':
                            recv_data=s.recv(self.buffer)
                        else:
                            recv_data=ser.read(self.buffer)

                        recv_data=recv_data.replace('\n','')
                        if recv_data=='':
                            time.sleep(0.5)
                            recv_data=ser.read(self.buffer)
                            
                        
                        time_recv1=time.time()
                        time_diff1=time_recv1-time_send1
                        #self.logger('Logger.txt',data.replace('\n','')+'\t\t'+recv_data.replace('\n','')+'\n')
                        
                        if data!='':
                            if len(send_data.replace('\n',''))==len(data.replace('\n','')) and send_data.find(data)!=-1 :
                                count_pct_rcv=count_pct_rcv+1
                        comp_data=list(d.compare(send_data.splitlines(),data.splitlines()))
                        #for item in comp_data:
                        if ''.join(comp_data).rfind('-',0,1)!=-1 or ''.join(comp_data).rfind('+',0,1)!=-1:
                            for i in range(0,len(''.join(send_data)),100):
                                self.logger('Logger.txt','Send: '+str(count_pckt)+' '+''.join(send_data.splitlines())[i:100+i]+'\n')
                            for i in range(0,len(''.join(data)),100):
                                self.logger('Logger.txt','Received: '+str(count_pckt)+' '+''.join(data.splitlines())[i:100+i]+'\n')
                        
                                
                        
                        if recv_data!='':
                            if len(send_data.replace('\n',''))==len(recv_data.replace('\n','')) and send_data.find(recv_data)!=-1:
                                count_pct_rcv1=count_pct_rcv1+1
                        comp_data=list(d.compare(send_data.splitlines(),recv_data.splitlines()))
                        #for item in comp_data:
                        if ''.join(comp_data).rfind('-',0,1)!=-1 or ''.join(comp_data).rfind('+',0,1)!=-1:
                            for i in range(0,len(''.join(send_data)),100):
                                self.logger('Logger.txt','\t'*29+'Send: '+str(count_pckt)+' '+''.join(send_data.splitlines())[i:100+i]+'\n')
                            for i in range(0,len(''.join(recv_data)),100):
                                self.logger('Logger.txt','\t'*29+'Received: '+str(count_pckt)+' '+''.join(recv_data.splitlines())[i:100+i]+'\n')

                        time_diff=time_received-time_send
                        list_time.append(round(time_diff,2))
                        list_time1.append(round(time_diff1,2))
                        print '\n'
                        print 'Difference time pkt send/pkt received: '+str(time_diff)
                        print '\n'
                        #time.sleep(2)            
                    loss_pkt=count_pckt-count_pct_rcv
                    loss_pkt1=count_pckt-count_pct_rcv1
                    if count_pct_rcv!=0:
                        ratio_loss=float(loss_pkt)/float(count_pct_rcv)
                    else:
                        ratio_loss=100
                    if count_pct_rcv1!=0:
                        ratio_loss1=float(loss_pkt1)/float(count_pct_rcv1)
                    else:
                        ratio_loss1=100
                    print 'Send pkt: '+str(count_pckt)+' Received pkt: '+str(count_pct_rcv)+' Loss packets: '+str(loss_pkt) 
                    print 'Packet loss ratio is: '+str(round(ratio_loss,2))
                    #print 'Throughput value: '+str(Throughput.getThroughput(self.host))+' Mbps'
                    self.logger('Logger.txt','\n')
                    self.logger('Logger.txt','='*200+'\n')
                    self.logger('Logger.txt','Average time to receive '+self.iterations+' from serial to tcp '+str(round(sum(list_time) / float(len(list_time)),2))+' seconds'+'\t'*11+'Average time to receive '+self.iterations+' from serial to tcp '+str(round(sum(list_time) / float(len(list_time)),2))+' seconds'+'\n')
                    self.logger('Logger.txt','Send pkt: '+str(count_pckt)+' Received pkt: '+str(count_pct_rcv)+' Loss packets: '+str(loss_pkt)+'\t'*16+'Send pkt: '+str(count_pckt)+' Received pkt: '+str(count_pct_rcv1)+' Loss packets: '+str(loss_pkt1)+'\n')
                    self.logger('Logger.txt','Packet loss ratio is: '+str(round(ratio_loss,2))+'%'+'\t'*20+'Packet loss ratio is: '+str(round(ratio_loss1,2))+'%'+'\n')
                    '''self.logger('Logger.txt','Throughput value: '+str(Throughput.getThroughput(self.host))+' Mbps'+'\n')'''
                    #self.logger('Logger.txt','='*200+'\n')
                    ser.close()
                    s.close()
                #ser.close()

            except Exception, e1:
                print e1.__doc__
                print e1.message
                print "error communicating...: " + str(e1)


            
                         

    def read_testsuite(self):

    
        testlist  = open('TestSuiteConnect.csv', "rb")
        reader = csv.reader(testlist)
        rownum=0
        
        for row in reader:
            if rownum==0:
                rownum+=1
                continue        
            else:
                self.TestId=row[0]
                self.TestStatus=row[1]
                if self.TestId.startswith('#')==True:
                    continue
                if 'TRUE' in self.TestStatus: 
                    self.sport=row[2]
                    self.comport=row[3]
                    self.interface=row[4]
                    self.srate=row[5]
                    self.sbytesize=row[6]
                    self.sparity=row[7 ]
                    self.sstopbits=row[8]
                    self.sxonxoff=row[9]
                    self.srtscts=row[10]
                    self.sdsrdtr=row[11]
                    self.host=row[12]
                    self.tcport=row[13]
                    self.packlength=row[14]
                    self.iterations=row[15]
                    self.filesend=row[16]
                else:
                    continue
                

                self.sendreceive(self.interface)


                # tcp settings

        
def main():
    file = open('Logger.txt', "w")
    file.close()
    inst=Connect()
    inst.read_testsuite()
    del inst
    
if __name__ == "__main__": main()
