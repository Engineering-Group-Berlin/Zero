package de.egb.debugstream;

import com.fasterxml.jackson.databind.ObjectMapper;


import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;

public final class JSONFrameWriter implements Closeable {
    private final ObjectMapper mapper = new ObjectMapper();
    private final BufferedWriter out;

    public JSONFrameWriter(Path file, boolean append) throws IOException {
        Path abs = file.toAbsolutePath();
        Path parent = abs.getParent();
        if (parent != null) Files.createDirectories(parent);

        out = new BufferedWriter(new OutputStreamWriter(
                new FileOutputStream(abs.toFile(), append),
                StandardCharsets.UTF_8
        ));
    }

    public void write(Object frame) throws IOException {
        out.write(mapper.writeValueAsString(frame));
        out.write('\n');
    }

    public void flush() throws IOException
    {
        out.flush();
    }

    @Override public void close() throws IOException
    {
        out.close();
    }
}
