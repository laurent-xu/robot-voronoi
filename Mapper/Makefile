help:
	@echo Use 'make install' to install Mapper3 into /usr/local.
install: install-Mapper3
uninstall: uninstall-Mapper3

# Installation root directory, you can override in the shell 
ifndef DESTDIR
DESTDIR=
endif


# Where to install things:
mapper3_dir = $(DESTDIR)/usr/local/Mapper3
mapper3basic_dir = $(DESTDIR)/usr/local/Mapper3Basic
mapper3pro_dir = $(DESTDIR)/usr/local/MobilePlanner
sharedir = $(mapper3_dir)
bindir = $(mapper3_dir)/bin
docdir = $(sharedir)/doc
sharedir_basic = $(mapper3basic_dir)
bindir_basic = $(mapper3basic_dir)/bin
docdir_basic = $(mapper3basic_dir)/doc
sharedir_pro = $(mapper3pro_dir)
bindir_pro = $(mapper3pro_dir)/bin
docdir_pro = $(mapper3pro_dir)/doc
sysbindir = $(DESTDIR)/usr/local/bin
desktop_entry_dir = $(DESTDIR)/usr/share/applications

MAPPER3_DIST_FILES_REQUIRED_BASE=deploy/Mapper3_Changes.txt deploy/LICENSE.txt deploy/MAPPER3_README.txt 
MAPPER3BASIC_DIST_FILES_REQUIRED_BASE=deploy/Mapper3Basic_Changes.txt deploy/LICENSE.txt deploy/MAPPER3_README.txt
MOBILEPLANNER_DIST_FILES_REQUIRED_BASE=deploy/MobilePlanner_Changes.txt deploy/LICENSE.txt deploy/MOBILEPLANNER_README.txt 

MAPPER3_DIST_FILES_REQUIRED_LINUX=$(MAPPER3_DIST_FILES_REQUIRED_BASE) bin/Mapper3 deploy/Mapper3.desktop src/mapper_large.png  src/mapper_small.png
MAPPER3BASIC_DIST_FILES_REQUIRED_LINUX=$(MAPPER3BASIC_DIST_FILES_REQUIRED_BASE) bin/Mapper3Basic deploy/Mapper3Basic.desktop src/mapper_large.png src/mapper_small.png
MOBILEPLANNER_DIST_FILES_REQUIRED_LINUX=$(MOBILEPLANNER_DIST_FILES_REQUIRED_BASE) bin/MobilePlanner deploy/MobilePlanner.desktop src/mapper_large.png src/mapper_small.png

MAPPER3_DIST_FILES_REQUIRED_WIN=$(MAPPER3_DIST_FILES_REQUIRED_BASE)
MAPPER3BASIC_DIST_FILES_REQUIRED_WIN=$(MAPPER3BASIC_DIST_FILES_REQUIRED_BASE)
MOBILEPLANNER_DIST_FILES_REQUIRED_WIN=$(MOBILEPLANNER_DIST_FILES_REQUIRED_BASE)


install-Mapper3: $(MAPPER3_DIST_FILES_REQUIRED_LINUX) 
	 install -d $(bindir) $(docdir) $(sharedir) $(sysbindir)
	 install --strip -m 755 bin/Mapper3 $(bindir)/Mapper3
	 find icons -path \*CVS -prune -o -type f -exec install -D -m 644 {} $(sharedir)/{} \;
	 install -D -m 644 deploy/Mapper3.desktop  $(desktop_entry_dir)/Mapper3.desktop  
	 install -D -m 644 deploy/MAPPER3_README.txt $(docdir)/README.txt
	 install -D -m 644 deploy/LICENSE.txt $(docdir)/LICENSE.txt
	 install -D -m 644 deploy/Mapper3_Changes.txt $(docdir)/Changes.txt
	 install -D -m 644 src/mapper_large.png $(sharedir)/icons/mapper3_icon.png
	 install -D -m 644 src/mapper_small.png $(sharedir)/icons/default/editor_logo.png
	 ln -sf /usr/local/Mapper3/bin/Mapper3 $(sysbindir)/Mapper3

install-Mapper3Basic: $(MAPPER3BASIC_DIST_FILES_REQUIRED_LINUX) 
	 install -d $(sysbindir) $(bindir_basic) $(docdir_basic) $(sharedir_basic)
	 install --strip -m 755 bin/Mapper3Basic $(bindir_basic)/Mapper3Basic
	 find icons -path \*CVS -prune -o -type f -exec install -D -m 644 {} $(sharedir_basic)/{} \;
	 install -D -m 644 deploy/Mapper3Basic.desktop  $(desktop_entry_dir)/Mapper3Basic.desktop
	 install -D -m 644 deploy/MAPPER3_README.txt $(docdir_basic)/README.txt
	 install -D -m 644 deploy/LICENSE.txt $(docdir_basic)/LICENSE.txt
	 install -D -m 644 deploy/Mapper3Basic_Changes.txt $(docdir_basic)/Changes.txt
	 install -D -m 644 src/mapper_large.png $(sharedir_basic)/icons/mapper3_icon.png
	 install -D -m 644 src/mapper_small.png $(sharedir_basic)/icons/default/editor_logo.png
	 ln -sf /usr/local/Mapper3Basic/bin/Mapper3Basic $(sysbindir)/Mapper3Basic

install-MobilePlanner: $(MOBILEPLANNER_DIST_FILES_REQUIRED_LINUX)
	 install -d $(sysbindir) $(bindir_pro) $(docdir_pro) $(sharedir_pro)
	 install --strip -m 755 bin/MobilePlanner $(bindir_pro)/MobilePlanner
	 find icons -path \*CVS -prune -o -type f -exec install -D -m 644 {} $(sharedir_pro)/{} \;
	 install -D -m 644 deploy/MobilePlanner.desktop  $(desktop_entry_dir)/MobilePlanner.desktop
	 install -D -m 644 deploy/MOBILEPLANNER_README.txt $(docdir_pro)/README.txt
	 install -D -m 644 deploy/LICENSE.txt $(docdir_pro)/LICENSE.txt
	 install -D -m 644 deploy/MobilePlanner_Changes.txt $(docdir_pro)/Changes.txt
	 install -D -m 644 src/mapper_large.png $(sharedir_pro)/icons/mobileplanner_icon.png
	 install -D -m 644 src/mapper_small.png $(sharedir_pro)/icons/default/editor_logo.png
	 ln -sf /usr/local/MobilePlanner/bin/MobilePlanner $(sysbindir)/MobilePlanner


uninstall-Mapper3:
	 rm -rf $(DESTDIR)/usr/local/Mapper3
	 rm -f  $(sysbindir)/Mapper3
	 rm -f  $(desktop_entry_dir)/Mapper3.desktop

uninstall-Mapper3Basic:
	 rm -rf $(DESTDIR)/usr/local/Mapper3Basic
	 rm -f  $(sysbindir)/Mapper3Basic
	 rm -f  $(desktop_entry_dir)/Mapper3Basic.desktop

uninstall-MobilePlanner:
	 rm -rf $(DESTDIR)/usr/local/MobilePlanner
	 rm -f  $(sysbindir)/MobilePlanner
	 rm -f  $(desktop_entry_dir)/MobilePlanner.desktop


