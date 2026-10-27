package view;

import javax.swing.*;
import java.awt.*;

public class SSAlgorithm extends JFrame {

    public SSAlgorithm() {
        setTitle("Salp Swarm Algorithm");
        setSize(1200, 800);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setLocationRelativeTo(null);

        MapPanel mapPanel = new MapPanel();
        add(mapPanel, BorderLayout.CENTER);

        JPanel controlPanel = new JPanel();
        controlPanel.setLayout(new FlowLayout());

        JButton runBtn = new JButton("Запустить SSA");
        JButton flyBtn = new JButton("Начать полет");
        flyBtn.setEnabled(false);

        JCheckBox cbUav1 = new JCheckBox("БПЛА 1", true);
        JCheckBox cbUav2 = new JCheckBox("БПЛА 2", true);
        JCheckBox cbUav3 = new JCheckBox("БПЛА 3", true);

        cbUav1.addActionListener(e -> { mapPanel.showUav[0] = cbUav1.isSelected(); mapPanel.repaint(); });
        cbUav2.addActionListener(e -> { mapPanel.showUav[1] = cbUav2.isSelected(); mapPanel.repaint(); });
        cbUav3.addActionListener(e -> { mapPanel.showUav[2] = cbUav3.isSelected(); mapPanel.repaint(); });

        runBtn.addActionListener(e -> {
            runBtn.setEnabled(false);
            flyBtn.setEnabled(false);
            mapPanel.stopAnimation();
            runBtn.setText("Идет оптимизация...");

            new Thread(() -> {
                mapPanel.runSSA();
                SwingUtilities.invokeLater(() -> {
                    runBtn.setEnabled(true);
                    flyBtn.setEnabled(true);
                    runBtn.setText("Запустить SSA");
                    mapPanel.repaint();
                });
            }).start();
        });

        flyBtn.addActionListener(e -> mapPanel.startAnimation());

        controlPanel.add(runBtn);
        controlPanel.add(flyBtn);
        controlPanel.add(new JLabel(" | "));
        controlPanel.add(cbUav1);
        controlPanel.add(cbUav2);
        controlPanel.add(cbUav3);

        add(controlPanel, BorderLayout.SOUTH);
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> new SSAlgorithm().setVisible(true));
    }
}