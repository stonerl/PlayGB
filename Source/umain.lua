import "util"

ROMS = {}

function register_rom(t)
    for i, title in wipairs(t.title) do
        ROMS[string.trim(title)] = t
    end
end

function on_load_rom(title)
    title = string.trim(title)
    print("rom loaded: ", title)
    local rom = ROMS[title]
    if not rom then return end
    
    print("special rom!")
    if rom.on_load then
        rom.on_load()
    end
end