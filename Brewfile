# source OpenSTA brewfile
eval(IO.read(File.join(File.dirname(__FILE__), 'third_party', 'OpenSTA', 'Brewfile')), binding)

tap "silimate/silimate"

brew "readline"
brew "silimate/silimate/tcl-readline", trusted: true

brew "libelf"
brew "dwarfutils"
