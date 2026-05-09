import { DashboardData } from '../types/types';

const FETCH_INTERVAL = 1000; // 1 second refresh rate

export async function fetchDashboardData(): Promise<DashboardData> {
  try {
    const response = await fetch('/database/whole.json');
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }
    return await response.json();
  } catch (error) {
    console.error('Error fetching dashboard data:', error);
    throw error;
  }
}