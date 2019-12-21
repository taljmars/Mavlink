package com.dronegcs.mavlink.spring;

import com.dronegcs.mavlink.is.protocol.msg_metadata.enums.MAV_PARAM_COPTER;
import com.dronegcs.mavlink.is.protocol.msg_metadata.enums.MAV_PARAM_I;
import com.dronegcs.mavlink.is.protocol.msg_metadata.enums.MAV_PARAM_PLANE;
import com.opencsv.CSVWriter;

import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class GenerateParametersGithubDoc {

    public static void main(String[] args) throws Exception {
        FileWriter fileWriter = new FileWriter("README.md");

        fileWriter.write("ArduCopter Parameters");
        fileWriter.write("\n");
        CSVWriter csvWriter = new CSVWriter(fileWriter,'|');
        appendParameters(csvWriter, MAV_PARAM_COPTER.values());

        fileWriter.write("\n");
        fileWriter.write("\n");

        fileWriter.write("ArduPilot Parameters");
        fileWriter.write("\n");
        appendParameters(csvWriter, MAV_PARAM_PLANE.values());

        csvWriter.close();
        return;
    }

    private static void appendParameters(CSVWriter csvWriter, MAV_PARAM_I[] params) {
        csvWriter.writeNext(new String[]{"Name "," Possible Value "," Increment "," Unit "," Range ", " Read Only ", " Title " ," Description"}, false);
        csvWriter.writeNext(new String[]{"--- "," --- "," --- "," --- "," --- ", " ---", " --- " ," ---"},false);
        for (MAV_PARAM_I val : params) {
            List<String> line = new ArrayList();
            line.add(val.getName() + " ");
            line.add(" " + val.getDefaultValue() + " ");
            line.add(" " + val.getIncrement() + " ");
            line.add(" " + val.getUnit() + " ");
//            line.add(val) //range
            if (val.getRange() != null) {
                line.add(" " + val.getRange().getMin() + " " + val.getRange().getMax() + " ");
            }
            else if (val.getOptions() != null) {
                String optionString = " ";
                for (Map.Entry<Integer, String> option : val.getOptions().entrySet()) {
                    optionString += option.getKey() + ":" + option.getValue() + " ";
                }
                line.add(optionString + " ");
            }
            else {
                line.add(" ");
            }
            line.add(" " + val.isReadOnly()  + " ");
            line.add(" " + val.getTitle() + " ");
            line.add(" " + val.getDescription());
            String[] res = line.toArray(new String[0]);
            csvWriter.writeNext(res,false);
        }
    }

}
