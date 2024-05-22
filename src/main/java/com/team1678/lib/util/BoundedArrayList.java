package com.team1678.lib.util;

import java.util.ArrayList;

public class BoundedArrayList<T> extends ArrayList<T> {

	private final int maxSize;

	public BoundedArrayList(int maxSize) {
		super();
		this.maxSize = maxSize;
	}

	@Override
	public boolean add(T element) {
		if (size() >= maxSize) {
			remove(0); // Remove the oldest element
		}
		return super.add(element);
	}
}
