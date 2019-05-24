%% Sarthak Sharma.
% Takes tracklets for a sequence and also id's,return information of object with those id's.

function info = getInfo(tr,id)

info={};
itt=1;
for obj_idx=1:numel(tr)
    % A struct with various feilds.
    p=tr(obj_idx);
    o_id=p.id;
    if o_id == -1
        continue
    end
    if  ( strcmp(p.type,'Car') && o_id == id)
        % Return the bbox ,ry and 3D
        info{itt}=[p.x1;p.y1;p.x2;p.y2;p.ry;];
        itt=itt+1;
    end
    
end

end