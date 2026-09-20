## Projekt Struktúra:


### `src`
A projekt fő forráskódja, src-elrendezési mintát követve. Tartalmazza az összes Python modult és csomagot,
valamint egy szimbolikus linket az OpenRocket Java kódjához.

### `dat`
Adatkönyvtár, amely tartalmazza:
- OpenRocket tervezési fájlokat (`.ork`)
- Szimulációs eredményeket (kimeneti fájlok és grafikonok)

### `docs`
Projekt dokumentáció, Sphinx-szel építve, beleértve:
- API dokumentációt
- Használati útmutatókat
- Konfigurációs fájlokat

### `out`
Külső függőségek és fordított elemek:
- OpenRocket JAR fájl
- Egyéb bináris függőségek

### Gyökérkönyvtár Fájlok
- `pyproject.toml`: Projekt konfiguráció és függőségek
- `LICENSE`: Projekt licenc
- `README.md`: Projekt áttekintés és dokumentáció
- `.gitignore`: Git kizárási minták