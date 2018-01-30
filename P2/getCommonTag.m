function common_tag_id = getCommonTag(tag_ids_first, tag_ids_next)
common_tag_found = false;
for i = 1: size(tag_ids_next,1)
    if common_tag_found
        break;
    end
    for j = 1:size(tag_ids_first,1)
        if tag_ids_next(i) == tag_ids_first(j)
            common_tag_id = tag_ids_next(i);
            common_tag_found = true;
            break;
        end
    end
end
end