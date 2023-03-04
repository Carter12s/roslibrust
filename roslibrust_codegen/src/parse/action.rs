use crate::parse::ParsedMessageFile;
use crate::Package;
use std::path::{Path, PathBuf};

use super::parse_ros_message_file;

#[derive(Clone, Debug)]
pub struct ParsedActionFile {
    pub name: String,
    pub package: String,
    pub action_type: ParsedMessageFile,
    pub goal_type: ParsedMessageFile,
    pub result_type: ParsedMessageFile,
    pub feedback_type: ParsedMessageFile,
    /// The contents of the action file this instance was parsed from
    pub source: String,
    /// The path where the message was found
    pub path: PathBuf,
}

pub fn parse_ros_action_file(
    data: &str,
    name: &str,
    package: &Package,
    path: &Path,
) -> ParsedActionFile {
    let mut dash_line_number_1 = None;
    let mut dash_line_number_2 = None;

    for (line_num, line) in data.lines().enumerate() {
        match (line.find("---"), line.find('#')) {
            (Some(dash_idx), Some(comment_idx)) => {
                if dash_idx < comment_idx {
                    if dash_line_number_1.is_none() {
                        dash_line_number_1 = Some(line_num);
                    } else {
                        dash_line_number_2 = Some(line_num);
                        break;
                    }
                }
            }
            (Some(_), None) => {
                if dash_line_number_1.is_none() {
                    dash_line_number_1 = Some(line_num);
                } else {
                    dash_line_number_2 = Some(line_num);
                    break;
                }
            }
            _ => continue,
        }
    }
    let str_accumulator = |mut acc: String, line: &str| -> String {
        acc.push_str(line);
        acc.push('\n');
        acc
    };

    if let (Some(first_dash_line), Some(second_dash_line)) =
        (dash_line_number_1, dash_line_number_2)
    {
        let goal_str = data
            .lines()
            .take(first_dash_line)
            .fold(String::new(), str_accumulator);
        let result_str = data
            .lines()
            .skip(first_dash_line + 1)
            .take(second_dash_line - first_dash_line)
            .fold(String::new(), str_accumulator);
        let feedback_str = data
            .lines()
            .skip(second_dash_line + 1)
            .fold(String::new(), str_accumulator);

        ParsedActionFile {
            name: name.to_owned(),
            package: package.name.clone(),
            action_type: generate_action_msg(name, package, path),
            goal_type: parse_ros_message_file(
                &goal_str,
                format!("{name}Goal").as_str(),
                package,
                path,
            ),
            result_type: parse_ros_message_file(
                &result_str,
                format!("{name}Result").as_str(),
                package,
                path,
            ),
            feedback_type: parse_ros_message_file(
                &feedback_str,
                format!("{name}Feedback").as_str(),
                package,
                path,
            ),
            source: data.to_owned(),
            path: path.to_owned(),
        }
    } else {
        panic!(
            "Failed to find both expected delimiter lines '---' in {}/{name}",
            &package.name
        )
    }
}

fn generate_action_msg(name: &str, package: &Package, path: &Path) -> ParsedMessageFile {
    let source = format!(
        r#"
{name}ActionGoal action_goal
{name}ActionResult action_result
{name}ActionFeedback action_feedback
    "#
    );

    parse_ros_message_file(&source, name, package, path)
}

fn generate_action_goal_msg(name: &str, package: &Package, path: &Path) -> ParsedMessageFile {
    let source = format!(
        r#"
Header header
actionlib_msgs/GoalID goal_id
{name}Goal goal
    "#
    );

    parse_ros_message_file(&source, name, package, path)
}

fn generate_action_result_msg(name: &str, package: &Package, path: &Path) -> ParsedMessageFile {
    let source = format!(
        r#"
Header header
actionlib_msgs/GoalStatus status
{name}Result result
        "#
    );

    parse_ros_message_file(&source, name, package, path)
}

fn generate_action_feedback_msg(name: &str, package: &Package, path: &Path) -> ParsedMessageFile {
    let source = format!(
        r#"
Header header
actionlib_msgs/GoalStatus status
{name}Feedback feedback
        "#
    );

    parse_ros_message_file(&source, name, package, path)
}
