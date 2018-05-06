package com.dronegcs.mavlink.is.drone.profiles;

public class ParameterDetail {

    private int generationIdx;
    private String name;
    private String unit;
    private String range;
    private String description;

    public ParameterDetail(int generationIdx, String name, String unit, String range, String description) {
        this.generationIdx = generationIdx;
        this.name = name;
        this.unit = unit;
        this.range = range;
        this.description = description;
    }

    public int getGenerationIdx() {
        return generationIdx;
    }

    public void setGenerationIdx(int generationIdx) {
        this.generationIdx = generationIdx;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public String getUnit() {
        return unit;
    }

    public void setUnit(String unit) {
        this.unit = unit;
    }

    public String getRange() {
        return range;
    }

    public void setRange(String range) {
        this.range = range;
    }

    public String getDescription() {
        return description;
    }

    public void setDescription(String description) {
        this.description = description;
    }

    @Override
    public String toString() {
        return "ParameterDetail{" +
                "idx=" + generationIdx +
                ", name='" + name + '\'' +
                ", unit='" + unit + '\'' +
                ", range='" + range + '\'' +
                ", description='" + description + '\'' +
                '}';
    }
}
