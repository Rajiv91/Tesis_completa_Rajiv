%function script()
%Puntos seleccionados en pixeles

imgPts = [815, 800, 780, 754, 957, 959, 957, 959, 1099, 1114, 1134, 1160;
	  	880, 924, 979, 1054, 878, 927, 979, 1057, 881, 928, 982, 1058;
	  	  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1];

scnPts = [-0.4, -0.4, -0.4, -0.4, 0, 0, 0, 0, 0.4, 0.4, 0.4, 0.4;
	 0, 0.4, 0.8, 1.2, 0, 0.4, 0.8, 1.2, 0, 0.4, 0.8, 1.2;
	  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
	  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1];	 	  	

K = [616.1640014648438, 0, 325.5280151367188;
  0, 616.8200073242188, 228.6600036621094;
  0, 0, 1];
  Kinv=inv(K);
  %se pasa a m
  imgPtsm=Kinv*imgPts;
  %La homografía es de scp a imp


  imp=[imgPts(1:2,:)];
  scp=[scnPts(1:2,:)];
  scp2=[scp;ones(1,12)];
H=[0.5803808811835411, -0.2598242943697432, 1.025510784977015;
  0.01386299401170919, -0.1030546714573874, 1.055581864549779;
  0.00716268752845311, -0.2539973142553691, 1];
  mapeoIm=H*scp2;
  %mapeoIm2
  %Deshomog...
  for i=1: size(mapeoIm,2)
    mapeoIm(:,i)=mapeoIm(1:3,i)/mapeoIm(3,i);
  end
%Distancia entre los 2 conjuntos de puntos
temp=mapeoIm;
dist1=[];
  for i=1: size(mapeoIm,2)
    temp(1:3,i)=(mapeoIm(1:3,i)-imgPtsm(1:3,i));
    dist1(i)=norm(temp(:,i));
  end

Dummy = [1, 0, 0, 0;
  0, 1, 0, 0;
  0, 0, 1, 0];

  %Antes de optimizar sin Gram
R= [0.9996387701129885, -0.6879449143636284, -0.01269168905964076;
  0.02387739970980576, -0.2728610783009988, 0.6637865969388977;
  0.01233689872251698, -0.6725166367827908, -0.2563361770259258];
T= [2.140332381160352;
  2.203093403559037;
  2.087089099904809];
  %Después de optimización
  #{
R=[0.2598969097806763, -0.6647438656771624, -0.7002606893228875;
  0.7380654434585482, -0.3308153385132648, 0.5879729389181061;
  -0.6225020812324402, -0.6697524682837988, 0.4045926185372987];
T=[2.017252607377037;
  2.838743747179702;
  2.15915031125904];

  #}

G= [[R T]; [0, 0, 0, 1]];
  
  %Rotamos y trasladamos 12 los puntos establecidos del rectángulo
  scnPtsRot=K*Dummy*G*scnPts;
  %Calculamos el error
  temp2=scnPtsRot;
  dist2=[];
  for i=1: size(imgPts,2)
    %temp2(1:4,i)=(intersec(1:4,i)-scnPtsRot(1:4,i));
    temp2(1:3,i)=scnPtsRot(1:3,i)/scnPtsRot(3,i);
    %dist2(i)=norm(temp2(:,i));
  end
  %temp2=[temp2(1:3,:)];
  temp3=temp2;
  errorPix=temp3;
  for i=1:size(temp3,2)
    errorPix(1:3,i)=(imgPts(1:3,i)-temp3(1:3,i));
  end

  %Error en metros
  scnPtsRotm=Dummy*G*scnPts;
%scnPtsRotm=[scnPtsRotm(1:3,:)];
%Deshomoge...
  for i=1: size(scnPtsRotm,2)
    scnPtsRotm(1:3,i)=scnPtsRotm(1:3,i)/scnPtsRotm(3,i);
  end

  errorM=scnPtsRotm;
  #{
  for i=1:size(errorM,2)
    errorM(1:3,i)=(imgPtsm(1:3,i)-scnPtsRotm(1:3,i));
  end
  #}

F=[0.0111224, 0.00086848, 0.000120477, 1.96727e-05, 4.02134e-06, 4.02134e-06, 4.02134e-06, 3.42637e-06, 3.42637e-06, 3.33294e-06, 3.22823e-06, 3.10635e-06, 2.98709e-06, 2.87315e-06, 2.76547e-06, 2.66384e-06, 2.56784e-06, 2.47701e-06, 2.39094e-06, 2.30928e-06, 2.23168e-06, 2.15787e-06, 2.08758e-06, 2.02055e-06, 1.95655e-06, 1.89538e-06, 1.83685e-06, 1.78079e-06, 1.72706e-06, 1.67553e-06, 1.62609e-06, 1.57865e-06, 1.53312e-06, 1.48946e-06, 1.4476e-06, 1.40751e-06, 1.36915e-06, 1.3325e-06, 1.29752e-06, 1.26422e-06, 1.23257e-06, 1.20256e-06, 1.17417e-06, 1.1474e-06, 1.12224e-06, 1.09867e-06, 1.07666e-06, 1.0562e-06, 1.03724e-06, 1.01969e-06, 1.00348e-06, 9.88666e-07, 9.75695e-07, 9.64844e-07, 9.55429e-07, 9.46695e-07, 9.38594e-07, 9.31468e-07, 9.25496e-07, 9.20635e-07, 9.16767e-07, 9.13764e-07, 9.11503e-07, 9.0987e-07, 9.08753e-07, 9.08046e-07, 9.07647e-07, 9.07459e-07, 9.07395e-07, 9.07383e-07, 9.07382e-07, 9.07382e-07, 9.07382e-07, 9.07382e-07 ];

g=[0.467148, 0.0930407, 0.00508172, 0.00197834, 0.000217561, 0.000217561, 0.000217561, 0.000434588, 0.000434588, 0.00159558, 0.00172886, 0.00168375, 0.00161391, 0.00153832, 0.00146525, 0.00139656, 0.00133255, 0.00127286, 0.00121725, 0.00116554, 0.0011175, 0.00107305, 0.00103205, 0.000994258, 0.000959496, 0.000927562, 0.000898067, 0.000873332, 0.000857811, 0.000844797, 0.000832481, 0.000820709, 0.00080921, 0.00079779, 0.000786397, 0.000774813, 0.000762803, 0.000750413, 0.000737423, 0.000723789, 0.000709477, 0.000694369, 0.000678506, 0.000661862, 0.000644473, 0.000626591, 0.000608313, 0.000590236, 0.000572759, 0.000555592, 0.000534085, 0.000495035, 0.000433336, 0.000381807, 0.000371361, 0.00037651, 0.000365191, 0.000337071, 0.000302421, 0.00026692, 0.00023261, 0.000199621, 0.000167914, 0.000137427, 0.000108157, 8.05071e-05, 5.49701e-05, 3.24279e-05, 1.45404e-05, 3.47316e-06, 1.54593e-07, 7.32873e-10, 7.32873e-10, 2.51595e-11 ];
plot(F);
  


%end
