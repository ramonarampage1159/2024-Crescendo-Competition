// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.Constants.ModulePosition.Swerve_Module_Position;

import java.util.*;


/**
 * Contains functions to convert {@link Map}s with {@link Swerve_Module_Position} keys to and from
 * arrays so that it's easier to use WPILib swerve functions.
 */
public class ModuleMap {

  /**
   * Creates a {@code Map} with {@link Swerve_Module_Position} keys from multiple values, in the
   * order specified in the {@link Swerve_Module_Position} enum.
   *
   * <p>For processing the output of a WPILib swerve function which returns an array.
   *
   * @param values Must have at least as many elements as {@link Swerve_Module_Position} has
   *     entries. Any entries after will be ignored.
   */
  @SafeVarargs
  public static <V> Map<Swerve_Module_Position, V> of(V... values) {
    Map<Swerve_Module_Position, V> map = new HashMap<>();
    for (int i = 0; i < Swerve_Module_Position.values().length; i++) {
      map.put(Swerve_Module_Position.values()[i], values[i]);
    }
    return map;
  }

  /**
   * Returns the values from a map as a {@link List} in the same order as in the {@link
   * Swerve_Module_Position} enum.
   *
   * <p>You can use this in a for/in loop without needing to supply an empty array like in {@link
   * #orderedValues(Map, Object[]) orderedValues}.
   */
  public static <V> List<V> orderedValuesList(Map<Swerve_Module_Position, V> map) {
    ArrayList<V> list = new ArrayList<>();
    for (Swerve_Module_Position i : Swerve_Module_Position.values()) {
      list.add(map.get(i));
    }
    return list;
  }

  /**
   * Returns the values from the map as an {@code Array} in the same order as in the {@link
   * Swerve_Module_Position} enum.
   *
   * <p>Useful when a WPILib swerve function requires an array as input.
   *
   * @param array An array of the class to output an array of, e.g. {@code
   *     moduleTranslations.valuesArray(new Translation2d[0])}. Required because Java can't make an
   *     array of generics.
   */
  public static <V> V[] orderedValues(Map<Swerve_Module_Position, V> map, V[] array) {
    return orderedValuesList(map).toArray(array);
  }
}
