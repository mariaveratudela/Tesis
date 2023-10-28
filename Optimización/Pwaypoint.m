function [posicion]=Pwaypoint (filename,drate,showflag)

if nargin < 1   % demo mode
    drate = 19; 
    filename  = 'testmap_883_556.png';
    showflag=1;
end

% read the input image and convert to a binary map
map=imread(filename);
mapshow=map;
%if (showflag)
%    figure,
%    imshow(map);
%end
map=rgb2gray(map);
map=logical(map);
imgcol=size(map,2);
imgrow=size(map,1);
row=round(imgrow/drate); 
col=round(imgcol/drate); 
cubsize=floor(drate/2);

%% -------------<Create the spars matrix of map>---------------
fprintf('discretizing the map...\n');
rowindx=[-1,-1,-1,0,1,1,1,0];
colindx=[-1,0,1,1,1,0,-1,-1];
cnt=1;
posiblewaypoint=1;
for i=0:row-1
    for j=0:col-1
        dist1=0;dist2=0;dist3=0;dist4=0;dist5=0;dist6=0;dist7=0;dist8=0;
        if (Checkobs(map,i,j,cubsize,drate)~=0)
            posiblewaypoint=posiblewaypoint+1;
            indx1=(j+1)+(col*(i));
            point(1:3,indx1)=[i;j;indx1]; %x,y,value
            cnttemp=0;
            for p=1:8
                r=i+rowindx(p);
                c=j+colindx(p);
                cnttemp=cnttemp+1;
                
                % check whether the cell is occupied by obstacle or it is out
                % of range 
                if(r>=0 && r<row && c>=0 && c<col && Checkobs(map,r,c,cubsize,drate)~=0)
                    indx2=((r)*col)+(c+1);
                    if(indx1~=indx2)
                        if (cnttemp==1)
                            r1=r;
                            c1=c;
                            %Cuenta hasta que encuentre obstáculo en pos1
                            while Checkobs(map,r1,c1,cubsize,drate)~=0
                                dist1=dist1+1;
                                r1=r1-1;
                                c1=c1-1;
                            end
                            indx1Mat(1,cnt)=indx1;
                            indx1Mat(2,cnt)=indx2;
                            weight(cnt)=1.4;
                            SpMatrix(indx1,indx2)=1.4;
                            cnt=cnt+1;
                        elseif (cnttemp==2)
                            r2=r;
                            c2=c;
                            %Cuenta hasta que encuentre obstáculo en pos1
                            while Checkobs(map,r2,c2,cubsize,drate)~=0
                                dist2=dist2+1;
                                r2=r2-1;
                                c2=c2;
                            end
                            indx1Mat(1,cnt)=indx1;
                            indx1Mat(2,cnt)=indx2;
                            weight(cnt)=1;
                            SpMatrix(indx1,indx2)=1;
                            cnt=cnt+1;
                        elseif (cnttemp==3)
                            r3=r;
                            c3=c;
                            %Cuenta hasta que encuentre obstáculo en pos1
                            while Checkobs(map,r3,c3,cubsize,drate)~=0
                                dist3=dist3+1;
                                r3=r3-1;
                                c3=c3+1;
                            end
                            indx1Mat(1,cnt)=indx1;
                            indx1Mat(2,cnt)=indx2;
                            weight(cnt)=1.4;
                            SpMatrix(indx1,indx2)=1.4;
                            cnt=cnt+1;
                        elseif (cnttemp==4)
                            r4=r;
                            c4=c;
                            %Cuenta hasta que encuentre obstáculo en pos1
                            while Checkobs(map,r4,c4,cubsize,drate)~=0
                                dist4=dist4+1;
                                r4=r4;
                                c4=c4+1;
                            end
                            indx1Mat(1,cnt)=indx1;
                            indx1Mat(2,cnt)=indx2;
                            weight(cnt)=1;
                            SpMatrix(indx1,indx2)=1;
                            cnt=cnt+1;
                        elseif (cnttemp==5)
                            r5=r;
                            c5=c;
                            %Cuenta hasta que encuentre obstáculo en pos1
                            while Checkobs(map,r5,c5,cubsize,drate)~=0
                                dist5=dist5+1;
                                r5=r5+1;
                                c5=c5+1;
                            end
                            indx1Mat(1,cnt)=indx1;
                            indx1Mat(2,cnt)=indx2;
                            weight(cnt)=1.4;
                            SpMatrix(indx1,indx2)=1.4;
                            cnt=cnt+1;
                        elseif (cnttemp==6)
                            r6=r;
                            c6=c;
                            %Cuenta hasta que encuentre obstáculo en pos1
                            while Checkobs(map,r6,c6,cubsize,drate)~=0
                                dist6=dist6+1;
                                r6=r6+1;
                                c6=c6;
                            end
                            indx1Mat(1,cnt)=indx1;
                            indx1Mat(2,cnt)=indx2;
                            weight(cnt)=1;
                            SpMatrix(indx1,indx2)=1;
                            cnt=cnt+1;
                        elseif (cnttemp==7)
                            r7=r;
                            c7=c;
                            %Cuenta hasta que encuentre obstáculo en pos1
                            while Checkobs(map,r7,c7,cubsize,drate)~=0
                                dist7=dist7+1;
                                r7=r7+1;
                                c7=c7-1;
                            end
                            indx1Mat(1,cnt)=indx1;
                            indx1Mat(2,cnt)=indx2;
                            weight(cnt)=1.4;
                            SpMatrix(indx1,indx2)=1.4;
                            cnt=cnt+1;
                        elseif (cnttemp==8)
                            r8=r;
                            c8=c;
                            %Cuenta hasta que encuentre obstáculo en pos1
                            while Checkobs(map,r8,c8,cubsize,drate)~=0
                                dist8=dist8+1;
                                r8=r8;
                                c8=c8-1;
                            end
                            indx1Mat(1,cnt)=indx1;
                            indx1Mat(2,cnt)=indx2;
                            weight(cnt)=1;
                            SpMatrix(indx1,indx2)=1;
                            cnt=cnt+1;
                        end
                    else                                                                      
                        fprintf('else \n');
                    end
                end % if(r>=1 && r<row && c>=1 && c<col && map(r,c)~=0)
            end % p=1:8
        else
            indx1=(j+1)+(col*(i));
            point(1:3,indx1)=0; %x    
            
        end %if (map(i,j)~=1) it's not an obstacle
        ii=i*drate+drate/2;
        jj=j*drate+drate/2;
        puntox=point(1,:);%*3*imgrow/2/row;
        puntoy=point(2,:);%*3*imgcol/2/col;
        libre=find(puntox~=0);
        posicion.pointlibre=[puntox(libre)+1;puntoy(libre)+1];
        posicion.points=[puntox;puntoy];
        %posicion.d1(posiblewaypoint)=dist1*sqrt(3*imgcol/2/col+3*imgrow/2/row);
        posicion.d1(indx1)=dist1*sqrt((3*imgcol/2/col)^2+(3*imgrow/2/row)^2);
        posicion.d2(indx1)=dist2*3*imgcol/2/col;
        posicion.d3(indx1)=dist3*sqrt((3*imgcol/2/col)^2+(3*imgrow/2/row)^2);
        posicion.d4(indx1)=dist4*3*imgrow/2/row;
        posicion.d5(indx1)=dist5*sqrt((3*imgcol/2/col)^2+(3*imgrow/2/row)^2);
        posicion.d6(indx1)=dist6*3*imgcol/2/col;
        posicion.d7(indx1)=dist7*sqrt((3*imgcol/2/col)^2-(3*imgrow/2/row)^2);
        posicion.d8(indx1)=dist8*3*imgrow/2/row;;
    end%end for col  number
end%end first for row


%if (showflag)
%    imshow(mapshow);
%end

% conver to a sparce matrix
[I,J,S] = find(SpMatrix);
[m n]=size(SpMatrix);
DG = sparse(I,J,S,m,n);

% output graph
posicion.g=DG;
posicion.map=map;
posicion.mapshow=mapshow;
posicion.drate=drate;

