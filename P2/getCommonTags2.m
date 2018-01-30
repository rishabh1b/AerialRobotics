function [common_tag_ids, uncommon_tag_ids] = getCommonTags2(tag_ids_first, tag_ids_next, knownTagIds)
common_tag_ids = [];
uncommon_tag_ids = [];
common_found = false;
for i = 1: size(tag_ids_next,1)
    for j = 1:size(tag_ids_first,1)
        if tag_ids_next(i) == tag_ids_first(j) || knownTagIds(tag_ids_next(i))
            common_tag_ids = [common_tag_ids;tag_ids_next(i)];
            common_found = true;
            break;
        end
    end
    if ~common_found
        uncommon_tag_ids = [uncommon_tag_ids;tag_ids_next(i)];
    end
    common_found = false;
end
end