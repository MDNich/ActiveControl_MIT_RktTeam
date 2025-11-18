    /**
     * Alapértelmezett konstruktor, amely alapértelmezett értékekkel hoz létre egy AirbrakeSet objektumot.
     * Az alapértelmezett értékek:
     * - length: 0.025 m
     * - width: 0.01 m
     * - thickness: 0.005 m
     * - CG_offset: 0.05 m
     * - indivAirbrakeMass: 0.023 kg
     * - numAirbrakes: 4
     */
    public AirbrakeSet() {
        this(0.025, 0.01, 0.005, 0.05, 0.023, 4);
    }

    /**
     * Konstruktor, amely a megadott paraméterekkel hoz létre egy AirbrakeSet objektumot.
     *
     * @param length a légellenállási fék hossza (y irány, radiális) méterben
     * @param width a légellenállási fék szélessége (z irány, tangenciális) méterben
     * @param thickness a légellenállási fék vastagsága (x irány, axiális) méterben
     * @param CG_offset a tömegközéppont távolsága a rakéta tengelyétől méterben
     * @param indivAirbrakeMass egy légellenállási fék tömege kilogrammban
     * @param numAirbrakes a légellenállási fékek száma
     */
    public AirbrakeSet(double length, double width, double thickness, double CG_offset, double indivAirbrakeMass, int numAirbrakes) {
        super(AxialMethod.TOP);
        this.setLength(length);
        this.setWidth(width);
        this.setThickness(thickness);
        this.setCG_offset(CG_offset);
        this.setIndivAirbrakeMass(indivAirbrakeMass);
        this.setNumAirbrakes(numAirbrakes);
        fireComponentChangeEvent(ComponentChangeEvent.AEROMASS_CHANGE);
    }

    /**
     * Visszaadja a komponens térfogatát.
     *
     * @return a légellenállási fék térfogata köbméterben (hossz × szélesség × vastagság)
     */
    @Override
    public double getComponentVolume() {
        return length * width * thickness;
    }

    /**
     * Visszaadja a komponens tömegközéppontját.
     *
     * @return a tömegközéppont koordinátái a komponens referenciakeretében
     */
    @Override
    public Coordinate getComponentCG() {
        Coordinate root = this.getPosition();
        return new Coordinate(root.x, root.y + CG_offset, root.z);
    }

    /**
     * Visszaadja a longitudinális egységnyi tehetetlenségi nyomatékot.
     * Ez az Iyy és Izz tehetetlenségi nyomatékok átlaga.
     *
     * @return a longitudinális egységnyi tehetetlenségi nyomaték (m²)
     */
    @Override
    public double getLongitudinalUnitInertia() {
        // return the average of Iyy and Izz
        double h = thickness;
        double l = length;
        double w = width;
        double d = CG_offset;
        return 0.5*((h*h+w*w)/12.0 + (l*l+h*h)/12.0 + d*d);
    }

    /**
     * Visszaadja a forgási egységnyi tehetetlenségi nyomatékot (Ixx).
     *
     * @return a forgási egységnyi tehetetlenségi nyomaték (m²)
     */
    @Override
    public double getRotationalUnitInertia() {
        // return Ixx
        double l = length;
        double w = width;
        double d = CG_offset;
        return d*d + (l*l + w*w)/12.0;
    }

    /**
     * Meghatározza, hogy a komponens engedélyez-e gyermek komponenseket.
     *
     * @return {@code false}, mert a légellenállási fék nem tartalmazhat gyermek komponenseket
     */
    @Override
    public boolean allowsChildren() {
        return false;
    }

    /**
     * Ellenőrzi, hogy a megadott komponens típus kompatibilis-e ezzel a komponenssel.
     *
     * @param type a vizsgálandó komponens osztály típusa
     * @return {@code false}, mert semmilyen komponens nem csatolható a légellenállási fékhez
     */
    @Override
    public boolean isCompatible(Class<? extends RocketComponent> type) {
        return false;
    }
