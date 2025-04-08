function wrap_table(s)
    if type(s) == "table" then
        return s
    else
        return {s}
    end
end

function wipairs(s)
    return ipairs(wrap_table(s))
end

function string.trim(s)
    return (s:gsub("^%s*(.-)%s*$", "%1"))
end