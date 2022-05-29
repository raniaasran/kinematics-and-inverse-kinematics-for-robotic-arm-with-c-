#include <iostream>
#include <math.h>

using namespace std;
const double l1=15.5;
const double l2=13 ;
const double l3=63;
const double l4=13;
const double l5=69;
const double d=20; //the distance between the 0 and ground
const double m1=24;//the midle one
const double m2=23.5;//the midle one
const double n1=22.5;// link Length trom the begining to m
const double n2=22.5;//ink Length trom the begining to m
const double r1=7.3;//
const double r2=7.7;


////////////////////////////////////////////////////////////////


double radian_to_degrees(double rad)
{
    return rad * 180 /M_PI ;
}
double degrees_to_radian(double deg)
{
    return deg * M_PI /180 ;
}


//////////////////////////////////////////////////////////////////////////////////


void invkinam (double* axis ,double* oldlin ,double* &ang,double* &puls)
{
double l ;
double x= axis[0];
double y= axis[1];
double z= axis[2]; //- d;
//geting theta3

if (x>0 && y==0)
{
 ang [2] =0 ;
 l=x;

}

else if (x>0 && y>0)
{
 ang [2] =atan(y/x) ;
 l=sqrt(x*x+y*y);

}
if (x==0 && y>0)
{
 ang [2] = M_PI/2 ;
  l=y;

}

else if (x<0 && y>0)
{
 ang [2] =M_PI- atan(y/(-1*x)) ;
  l=sqrt(x*x+y*y);
}
else if (x<0 && y==0)
{
 ang [2] =M_PI ;
  l=-1*x;

}

else if (x<0 && y<0)
{
 ang [2] =M_PI+ atan(y/x) ;
  l=sqrt(x*x+y*y);
}
if (x==0 && y<0)
{
 ang [2] = 3/2 *M_PI;
 l=-1*y;

}


else if (x>0 && y<0)
{
 ang [2] =2*M_PI- atan((-1*y)/x) ;
  l=sqrt(x*x+y*y);


}
//geting theta2
double a,b,c;
a=64.8;
b=70.7;
c=sqrt((l-l1)*(l-l1)+z*z);
ang[1] =acos((a*a+b*b-c*c)/(2*a*b));
ang[1]=ang[1]-atan(l2/l3)+atan(l5/l4);
//cout<<"theta2="<<radian_to_degrees(ang[1])<<endl;
if ((ang[1]<degrees_to_radian(185)) && (ang[1]>degrees_to_radian(85)))
{
    //geting theta1
     if(l>l1)
     {ang[0]=degrees_to_radian(185)-acos((c*c+a*a-b*b)/(2*a*c))-atan(z/(l-l1));}
     else if(l1>l)
     {ang[0]=atan(z/(l1-l))-acos((c*c+a*a-b*b)/(2*a*c))+degrees_to_radian(5);}
     else if(l==l1)
       {ang[0]=M_PI/2-acos((c*c+a*a-b*b)/(2*a*c));}
      // cout <<"theta1="<<radian_to_degrees(ang[0])<<endl;
}
else
 {
     c=sqrt((l+l1)*(l+l1)+z*z);
     ang[1] =acos((a*a+b*b-c*c)/(2*a*b));
     ang[1]=ang[1]-atan(l2/l3)+atan(l5/l4);
     ang[0]=atan(z/(l+l1))-acos((c*c+a*a-b*b)/(2*c*a));

 }
ang[0]=ang[0]+atan(l3/l2);
//cout <<"theta1="<<radian_to_degrees(ang[0])<<endl;
ang[0]=radian_to_degrees( ang[0]);
ang[1]=radian_to_degrees( ang[1]);
ang[2]=radian_to_degrees( ang[2]);




 if( ang[0]>90 && ang[0]<170&& ang[1]>90 && ang[1]<170)
    {

    double o1,o2,o3,o4,A,B;
     B=asin((23.6*sin(degrees_to_radian(ang[0])-atan(n1/r1)))/m1);
     // cout <<"b="<<radian_to_degrees(B) <<endl;
     A=M_PI-B-degrees_to_radian(ang[0])+atan(n1/r1);
     o1=(m1*sin(A))/(sin(degrees_to_radian(ang[0])-atan(n1/r1)));
    // cout <<"o1="<<o1<<endl;

     B=asin((23*sin(degrees_to_radian(ang[1])-atan(n2/r2)))/m2);
     // cout <<"b1="<<radian_to_degrees(B) <<endl;
     A=M_PI-B-degrees_to_radian(ang[1])+atan(n2/r2);
     o2=(m2*sin(A))/(sin(degrees_to_radian(ang[1])-atan(n2/r2)));
    //cout <<"b2=" <<radian_to_degrees(acos((23*23+o2*o2-m2*m2)/(2*23*o2)));
    //cout <<"o2="<<o2<<endl;

     o3=oldlin[0];
     //cout <<"o3="<<o3<<endl;

     o4=oldlin[1];
     //cout <<"o4="<<o4<<endl;

    puls[0]=(o1-o3)/0.4 ;
    puls[1]=(o2-o4)/0.4 ;
    puls[2]=(ang[2]-oldlin[2]) / 1.75;
    puls[0]=240* puls[0];
    puls[1]=240* puls[1];
    puls[2]=240*puls[2];
}
}

////////////////////////////////////////////////////////////////////////////////////////



void kinam (double* &ang, double*  &olin,double* dp ,double*& axis)
{
 //to know where we are we must know first the angels to work on it in kinematics
  double o1,o2,a;
   o1=((dp[0]/240)*0.4 )+olin[0];
   o2=((dp[1]/240)*0.4 )+olin[1];
 //  cout <<"dp"<<dp[1]/240<<endl;
   ang[0]=radian_to_degrees(acos((23.6*23.6+o1*o1-m1*m1)/(2*23.6*o1))+atan(n1/r1));
   ang[1]=radian_to_degrees(acos((23*23+o2*o2-m2*m2)/(2*23*o2))+atan(n2/r2));
   ang[2]=olin[2]+((dp[2]/240)*1.75);
   cout <<"ang0="<<ang[0]<<endl;
   cout <<"ang1="<<ang[1]<<endl;
   cout <<"ang2="<<ang[2]<<endl;
   olin[0]=o1;
   olin[1]=o2;
   olin[2]=ang[2];
  //cout <<"o2="<<o2<<endl;
//to get x and y and z

double l ;
double th1=ang[0]-5;
double th2=ang[1] ;
double th3=ang[2];
double phi=th1-90+180-th2 ;


l= l1 +l2 * cos(degrees_to_radian(180-th1)) - l3 *cos(degrees_to_radian(th1-90)) ;
axis[2] = l2 *sin (degrees_to_radian(180-th1)) + l3 * sin (degrees_to_radian(th1-90)) ;
if (th2>th1)
 {
   l = l- l4 *cos(degrees_to_radian( phi))+l5 *cos (degrees_to_radian(90-phi)) ;
   axis[2] = axis[2] + l4 *sin (degrees_to_radian(phi)) + l5 *sin(degrees_to_radian(90-phi)) ;
 }
else if (th1>th2)
 {
   l = l+ l4*cos( degrees_to_radian(180- phi)) +l5 *cos (degrees_to_radian(phi-90)) ;
   axis[2] = axis[2] + l4*sin (degrees_to_radian(180-phi)) + l5 *sin(degrees_to_radian(phi-90)) ;
 }
else if (th1==th2)
 {
   l=l+l5;
   axis[2] = axis[2] + l4 ;
 }
 if (l>0)
  {
      if (th3 <90 )
     {
      axis[0]= l*cos(degrees_to_radian(th3));
      axis[1]= l*sin(degrees_to_radian(th3));

     }

      else if (th3==90)
     {
      axis[0]= 0;
      axis[1]= l;
     }


     else if (th3>90 && th3<180 )
     {
      axis[0]= -1*l*cos(degrees_to_radian(180-th3));
      axis[1]= l*sin(degrees_to_radian(180-th3));

     }
     else if (th3==180)
     {
      axis[0]= -l;
      axis[1]= 0;
     }

     else if (th3>180 && th3<270 )
     {
      axis[0]= -1*l*cos(degrees_to_radian(th3-180));
      axis[1]= -1*l*sin(degrees_to_radian(th3-180));

     }
     else if (th3==270)
     {
      axis[0]= 0;
      axis[1]= -l;
     }

     else if (th3>270 && th3<360 )
     {
      axis[0]= l*cos(degrees_to_radian(360-th3));
      axis[1]= -1*l*sin(degrees_to_radian(360-th3));

     }


  }
  else if (l<0)
  {   l=-1*l;
      if (th3 <90 )
     {
      axis[0]= -1*l*cos(degrees_to_radian(90-th3));
      axis[1]= -1*l*sin(degrees_to_radian(th3));

     }

      else if (th3==90)
     {
      axis[0]= 0;
      axis[1]= -1*l;
     }


     else if (th3>90 && th3<180 )
     {
      axis[0]= -1*l*cos(degrees_to_radian(180-th3));
      axis[1]= l*sin(degrees_to_radian(180-th3));

     }
     else if (th3==180)
     {
      axis[0]= l;
      axis[1]= 0;
     }

     else if (th3>180 && th3<270 )
     {
      axis[0]= l*cos(degrees_to_radian(th3-180));
      axis[1]= l*sin(degrees_to_radian(th3-180));

     }
     else if (th3==270)
     {
      axis[0]= 0;
      axis[1]= l;
     }

     else if (th3>270 && th3<360 )
     {
      axis[0]= -1*l*cos(degrees_to_radian(360-th3));
      axis[1]= l*sin(degrees_to_radian(360-th3));

     }


  }



}


///////////////////////////////////////////////////////////////////////////////


int main()
{
double* point1 = new double[3];  //the point i want it to go to
double* point2 = new double[3];  //the point it already at
double* angles = new double[3];// the angiles which have been getten fron inversekinematics
double* currentangles = new double[3];
double* pls = new double[3];// the pulses it must take to go to this point
double* olldlin = new double[3];//
double* donepuls = new double[3];
//////////////////////////////////////////////////////////
//inverseinematics test
point1[0]=0;
point1[1]=25.7;
point1[2]=43.4;


olldlin[0]=45.3;
olldlin[1]=44.5;
olldlin[2]=0; //the old ang3
invkinam (point1 ,olldlin,angles,pls);

cout <<"theta1="<<angles[0]<<endl;
cout <<"theta2="<<angles[1]<<endl;
cout <<"theta3="<<angles[2]<<endl;
cout <<"number of pulses motor1="<<pls[0]/240<<endl;
cout <<"number of pulses motor2="<<pls[1]/240<<endl;
cout <<"number of pulses motor3="<<pls[2]/240<<endl;
//////////////////////////////////////////////////
//kinematics test


donepuls[0]=-17.0462*240;
donepuls[1]=-15.1552*240;
donepuls[2]=51.4286*240;

olldlin[0]=45.3;
olldlin[1]=44.5;
olldlin[2]=0;

kinam (currentangles, olldlin ,donepuls ,point2);

cout <<"where i am x="<<point2[0]<<endl;
cout <<"where i am y="<<point2[1]<<endl;
cout <<"where i am z="<<point2[2]<<endl;

/*donepuls[0]=-5*240;
donepuls[1]=0;
donepuls[2]=0;


kinam (currentangles, olldlin ,donepuls ,point2);
cout <<point2[0]<<endl;
cout <<point2[1]<<endl;
cout <<point2[2]<<endl;*/

    return 0;
}
