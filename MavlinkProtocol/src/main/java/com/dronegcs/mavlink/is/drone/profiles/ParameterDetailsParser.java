package com.dronegcs.mavlink.is.drone.profiles;

import com.opencsv.CSVReader;
import com.opencsv.CSVReaderBuilder;
import org.slf4j.LoggerFactory;
import org.springframework.core.io.ClassPathResource;

import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

/**
 * Created by taljmars on 6/17/2017.
 */
//@Component
public class ParameterDetailsParser {

    private final org.slf4j.Logger LOGGER = LoggerFactory.getLogger(ParameterDetailsParser.class);

    //    private static final String ParametersListFileName = "MavlinkParamsCopter";
//    private static final String ParametersListSeperator = "\\|\\|";
    private static final String ParametersListSeperator = ",";
    private static final Integer ParametersListTokens = 5;

//    private Map<String,ParameterDetail> parametersDetails;

//    @PostConstruct
//    public void init() {
//        parametersDetails = buildParser();
//    }

    public Map<String,ParameterDetail> parse(String filePath) throws IOException {

        Map<String,ParameterDetail> parametersDetails = new HashMap<String, ParameterDetail>();

        FileReader filereader = null;
        CSVReader csvReader = null;
        try {
            ClassPathResource resource = new ClassPathResource(filePath);

            // Create an object of file reader
            // class with CSV file as a parameter.
            filereader = new FileReader(resource.getURL().getPath());
            csvReader = new CSVReaderBuilder(filereader).withSkipLines(1).build();

            String[] tokens;

            // we are going to read data line by line
            int i = 0;
            while ((tokens = csvReader.readNext()) != null) {
                if (tokens.length != ParametersListTokens) { // + 1 because of the last empty section
                    System.err.println("Skipping Line: " + tokens);
                    continue;
                }

                String paramName = tokens[0];
                String paramUnit = tokens[2];
                String paramRange = tokens[3];
                String paramDescription = tokens[4];

                ParameterDetail parameterDetail = new ParameterDetail(i++, paramName, paramUnit, paramRange, paramDescription);
                System.out.println(parameterDetail);
                parametersDetails.put(paramName, parameterDetail);
            }

        }
        finally {
            if (csvReader != null)
                csvReader.close();

            if (filereader != null)
                filereader.close();
        }

        return parametersDetails;
    }
}
