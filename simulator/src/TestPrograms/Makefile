none:
	@echo
	@echo
	@echo "Linux Targets:"
	@echo "  maps_lin    - linux map test program (maps)"
	@echo "  lists_lin   - linux list test program (lists)"
	@echo "  clean_lin   - cleans linux object files and executables"
	@echo
	@echo "Windows Targets:"
	@echo "  maps_win    - windows map test program (maps.exe)"
	@echo "  lists_win   - windows list test program (lists.exe)"
	@echo "  clean_win   - cleans windows object files and executables"
	@echo
	@echo

lists_lin: liststest.c ../lists.c ../../include/lists.h
	gcc -Wall -I../../include/ -lm -o lists liststest.c ../lists.c

maps_lin: mapstest.c ../maps.c ../../include/maps.h
	gcc -Wall -I../../include/ -lm -o maps mapstest.c ../maps.c

lists_win: liststest.c ..\lists.c ..\..\include\lists.h
	bcc32 -elists.exe -I..\..\include\ liststest.c ..\lists.c

maps_win: mapstest.c ..\maps.c ..\..\include\maps.h
	bcc32 -emaps.exe -I..\..\include\ mapstest.c ..\maps.c

clean_lin:
	rm -rf lists maps

clean_win:
	del *.obj *.tds *.exe
