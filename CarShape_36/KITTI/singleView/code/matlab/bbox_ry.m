%% Sarthak Sharma.
% Takes tracklets for a sequence and also id's,return information of object with those id's.

function info = getInfo(tr,carID)

info={};
itt=1;
for obj_idx=1:numel(tr)
    % A struct with various feilds.
    p=tr(obj_idx);
    o_id=p.id;
    if o_id == -1
        continue
    end
    if  ( strcmp(p.type,'Car') && p.id == carID )
        % Return the bbox ,ry and 3D
        info{itt}=[p.x1;p.y1;p.x2;p.y2;p.ry;p.t(1);p.t(2);p.t(3)];
        itt=itt+1;
    end
    
end

end